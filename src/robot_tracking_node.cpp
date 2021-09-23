#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <bobi_vision/colour_detector.hpp>
#include <bobi_vision/mask_factory.hpp>

#include <bobi_vision/camera_config.hpp>

#include <cassert>

using namespace bobi;
using namespace bottom;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // get camera config from the parameter server
    CameraConfig camera_cfg = get_camera_config(nh);

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

    // check if we are using the setup with a robot tracker from the top camera
    bool robot_is_top;
    nh.param<bool>("robot_is_top", robot_is_top, false);

    // publisher for the image_transport wrapped image
    image_transport::Publisher raw_image_pub = it.advertise("bottom_camera/image_raw", 1);
    image_transport::Publisher undistorted_image_pub = it.advertise("bottom_camera/image_undistorted", 1);
    image_transport::Publisher masked_image_pub = it.advertise("bottom_camera/image_masked", 1);
    image_transport::Publisher blob_pub = it.advertise("bottom_camera/image_blobs", 1);

    // pose publisher
    ros::Publisher pose_pub;
    if (!robot_is_top) {
        pose_pub = nh.advertise<bobi_msgs::PoseVec>("robot_poses", 1);
    }

    cv::Mat distortion_coeffs = cv::Mat(1, 5, CV_64F);
    memcpy(distortion_coeffs.data, camera_cfg.distortion_coeffs.data(), camera_cfg.distortion_coeffs.size() * sizeof(double));
    cv::Mat camera_mat = cv::Mat(3, 3, CV_64F);
    memcpy(camera_mat.data, camera_cfg.camera_matrix.data(), camera_cfg.camera_matrix.size() * sizeof(double));

    cv::Rect roi;
    cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(camera_mat, distortion_coeffs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), 1., cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), &roi);

    bobi::MaskPtr setup_mask = bobi::MaskFactory()(camera_cfg.mask_type, camera_cfg.mask_specs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), CV_8UC3);

    bobi::ColourDetector cd(camera_cfg.led_colours, camera_cfg.hsv_thresholds);

    cv::Mat frame;
    cv::Mat frame_und;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        camera >> frame;

        cv::undistort(frame, frame_und, camera_mat, distortion_coeffs, new_camera_mat);
        frame_und = frame_und(roi);

        cv::Mat masked_frame = frame_und.clone();
        setup_mask->roi(masked_frame);

        if (!robot_is_top) {
            // detect robots
            std::vector<cv::Point3f> poses2d = cd.detect(masked_frame); // Detect colour blobs

            // publish the poses of the individuals that were detected
            bobi_msgs::PoseVec pv;
            for (const cv::Point3f& pose2d : poses2d) {
                bobi_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.xyz.x = pose2d.x * camera_cfg.pix2m;
                pose.pose.xyz.y = pose2d.y * camera_cfg.pix2m;
                pose.pose.rpy.yaw = pose2d.z;
                pv.poses.push_back(pose);
            }

            if (poses2d.size() > 0) {
                pose_pub.publish(pv);
            }
        }

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "bgr8", frame_und).toImageMsg();
        undistorted_image_pub.publish(undistorted_image_ptr);

        sensor_msgs::ImagePtr masked_image_ptr = cv_bridge::CvImage(header, "bgr8", masked_frame).toImageMsg();
        masked_image_pub.publish(masked_image_ptr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}