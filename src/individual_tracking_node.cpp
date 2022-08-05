#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_vision/blob_detector.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <bobi_vision/mask_factory.hpp>
#include <bobi_vision/camera_config.hpp>

#include <iostream>

using namespace bobi;
using namespace top;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "individual_tracking_node");
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

    // publisher for the image_transport wrapped image
    image_transport::Publisher raw_image_pub = it.advertise("top_camera/image_raw", 1);
    image_transport::Publisher undistorted_image_pub = it.advertise("top_camera/image_undistorted", 1);
    image_transport::Publisher masked_image_pub = it.advertise("top_camera/image_masked", 1);
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
    ROS_WARN("Undistorted image ROI set to: %d x %d", roi.size().width, roi.size().height);

    bobi::MaskPtr setup_mask = bobi::MaskFactory()(camera_cfg.mask_type, camera_cfg.mask_specs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), CV_8UC3);

    cv::Mat frame;
    cv::Mat frame_und;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        camera >> frame;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY); // enforce image_transport compatible grayscale

        cv::undistort(frame, frame_und, camera_mat, distortion_coeffs, new_camera_mat);
        frame_und = frame_und(roi);

        cv::Mat masked_frame = frame_und.clone();
        setup_mask->roi(masked_frame);

        // detect individuals
        std::vector<cv::Point3f> poses2d = bd.detect(masked_frame);

        std::vector<int> idcs_to_remove;
        for (size_t i = 0; i < poses2d.size(); ++i) {
            cv::patchNaNs(poses2d, -1);
            if (poses2d[i].x < 0 || poses2d[i].y < 0) {
                idcs_to_remove.push_back(i);
            }
        }

        for (size_t i = 0; i < idcs_to_remove.size(); ++i) {
            auto it = poses2d.begin();
            std::advance(it, idcs_to_remove[i] - i);
            poses2d.erase(it);
        }

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
        pose_pub.publish(pv);

        // publish the image that contains the filtered image
        sensor_msgs::ImagePtr blob_image_ptr = cv_bridge::CvImage(header, "mono8", bd.get_blob_frame()).toImageMsg();
        blob_pub.publish(blob_image_ptr);

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "mono8", frame_und).toImageMsg();
        undistorted_image_pub.publish(undistorted_image_ptr);

        sensor_msgs::ImagePtr masked_image_ptr = cv_bridge::CvImage(header, "mono8", masked_frame).toImageMsg();
        masked_image_pub.publish(masked_image_ptr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}