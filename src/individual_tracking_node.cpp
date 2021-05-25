#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_vision/blob_detector.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <iostream>

using namespace bobi;

struct TopCameraConfig {
    bool using_file = false;
    std::string camera_dev_no = "2";
    int camera_px_width = 512;
    int camera_px_height = 512;
    int fps = 30;
    std::vector<double> camera_matrix = {
        783.55455, 0., 256.11758,
        0., 783.21002, 262.14992,
        0., 0., 1.};
    std::vector<double> distortion_coeffs = {-0.176236, 0.418800, 0.005023, -0.002432, 0.000000};
};

TopCameraConfig get_camera_config(const ros::NodeHandle& nh)
{
    // configuration for the top camera
    TopCameraConfig top_camera;
    nh.param<bool>("top_camera/using_file", top_camera.using_file, top_camera.using_file);
    nh.param<std::string>("top_camera/camera_dev_no", top_camera.camera_dev_no, top_camera.camera_dev_no);
    nh.param<int>("top_camera/camera_px_width", top_camera.camera_px_width, top_camera.camera_px_width);
    nh.param<int>("top_camera/camera_px_height", top_camera.camera_px_height, top_camera.camera_px_height);
    nh.param<int>("top_camera/fps", top_camera.fps, top_camera.fps);
    nh.param<std::vector<double>>("top_camera/camera_matrix", top_camera.camera_matrix, top_camera.camera_matrix);
    nh.param<std::vector<double>>("top_camera/distortion_coefficients", top_camera.distortion_coeffs, top_camera.distortion_coeffs);
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

    if (camera_cfg.using_file) {
        camera = cv::VideoCapture(camera_cfg.camera_dev_no);
        medium = "Using video file: " + camera_cfg.camera_dev_no;
    }
    else {
        camera = cv::VideoCapture(std::stoi(camera_cfg.camera_dev_no), cv::CAP_GSTREAMER);
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
    image_transport::Publisher undistorted_image_pub = it.advertise("top_camera/image_undistorted", 1);
    image_transport::Publisher blob_pub = it.advertise("top_camera/image_blobs", 1);

    // pose publisher
    ros::Publisher pose_pub = nh.advertise<bobi_msgs::PoseVec>("naive_poses", 1);

    defaults::BlobDetectorConfig cfg;
    BlobDetector bd(cfg, true);

    cv::Mat distortion_coeffs = cv::Mat(1, 5, CV_64F);
    memcpy(distortion_coeffs.data, camera_cfg.distortion_coeffs.data(), camera_cfg.distortion_coeffs.size() * sizeof(double));
    cv::Mat camera_mat = cv::Mat(3, 3, CV_64F);
    memcpy(camera_mat.data, camera_cfg.camera_matrix.data(), camera_cfg.camera_matrix.size() * sizeof(double));

    cv::Rect roi;
    cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(camera_mat, distortion_coeffs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), 1., cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), &roi);

    cv::Mat frame;
    cv::Mat frame_und;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        camera >> frame;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY); // enforce image_transport compatible grayscale

        cv::undistort(frame, frame_und, camera_mat, distortion_coeffs, new_camera_mat);
        frame_und = frame_und(roi);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "mono8", frame_und).toImageMsg();
        undistorted_image_pub.publish(undistorted_image_ptr);

        // detect individuals
        std::vector<cv::Point3f> poses2d = bd.detect(frame_und);

        // publish the poses of the individuals that were detected
        bobi_msgs::PoseVec pv;
        for (const cv::Point3f& pose2d : poses2d) {
            bobi_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.xyz.x = pose2d.x;
            pose.pose.xyz.y = pose2d.y;
            pose.pose.rpy.yaw = pose2d.z;
            pv.poses.push_back(pose);
        }
        pose_pub.publish(pv);

        // publish the image that contains the filtered image
        sensor_msgs::ImagePtr blob_image_ptr = cv_bridge::CvImage(header, "mono8", bd.get_blob_frame()).toImageMsg();
        blob_pub.publish(blob_image_ptr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}