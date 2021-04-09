#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_vision/blob_detector.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/NaivePoses.h>

#include <iostream>

using namespace bobi;

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

struct TopCameraConfig {
    std::string camera_dev_no = "2";
    int camera_px_width = 512;
    int camera_px_height = 512;
    // pix2mm: 1.002142315;
    // annotate_image= true;
    // camera_matrix= [666.82179448,   0.,         340.53770393, 0., 665.59446379,
    // 252.23375323, 0., 0., 1.]; distortion_coeffs= [2.54816200e-02,
    // -9.99858067e-01,  2.86687084e-03, -3.54082085e-04, 3.05316286e+00];
    int fps = 30;
};

TopCameraConfig get_camera_config(const ros::NodeHandle& nh)
{
    // configuration for the top camera
    TopCameraConfig top_camera;
    nh.param<std::string>("top_camera/camera_dev_no", top_camera.camera_dev_no, top_camera.camera_dev_no);
    nh.param<int>("top_camera/camera_px_width", top_camera.camera_px_width, top_camera.camera_px_width);
    nh.param<int>("top_camera/camera_px_height", top_camera.camera_px_height, top_camera.camera_px_height);
    nh.param<int>("top_camera/fps", top_camera.fps, 30);
    return top_camera;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "individual_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // get camera config from the parameter server
    TopCameraConfig camera_cfg = get_camera_config(nh);

    // set opencv stream
    std::string medium;
    cv::VideoCapture camera;
    if (is_number(camera_cfg.camera_dev_no)) {
        camera = cv::VideoCapture(camera_cfg.camera_dev_no);
        medium = "Using video file: " + camera_cfg.camera_dev_no;
    }
    else {
        camera = cv::VideoCapture(camera_cfg.camera_dev_no, cv::CAP_GSTREAMER);
        medium = "Using camera device: " + camera_cfg.camera_dev_no;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_cfg.camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_cfg.camera_px_height);

    ROS_INFO("%s", medium.c_str());
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera/file");
        return 1;
    }

    // publisher for the image_transport wrapped image
    image_transport::Publisher raw_image_pub = it.advertise("top_camera/image_raw", 1);
    image_transport::Publisher blob_pub = it.advertise("top_camera/image_blobs", 1);

    // pose publisher
    ros::Publisher pose_pub;
    pose_pub = nh.advertise<bobi_msgs::NaivePoses>("naive_poses", 1);

    defaults::BlobDetectorConfig cfg;
    BlobDetector bd(cfg, true);
    cv::Mat frame;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        camera >> frame;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY); // enforce image_transport compatible grayscale

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        // detect individuals
        std::vector<cv::Point3f> poses2d = bd.detect(frame);

        // publish the poses of the individuals that were detected
        bobi_msgs::NaivePoses np;
        for (const cv::Point3f& pose2d : poses2d) {
            bobi_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.xyz.x = pose2d.x;
            pose.pose.xyz.y = pose2d.y;
            pose.pose.rpy.yaw = pose2d.z;
            np.poses.push_back(pose);
        }
        pose_pub.publish(np);

        // publish the image that contains the filtered image
        sensor_msgs::ImagePtr blob_image_ptr = cv_bridge::CvImage(header, "mono8", bd.get_blob_frame()).toImageMsg();
        blob_pub.publish(blob_image_ptr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}