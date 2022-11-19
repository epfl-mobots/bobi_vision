#ifndef BOBI_INDIVIDUAL_TRACKING_HPP
#define BOBI_INDIVIDUAL_TRACKING_HPP

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_vision/blob_detector.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <bobi_vision/mask_factory.hpp>
#include <bobi_vision/camera_config.hpp>

namespace bobi {
    class IndividualTracking {
    public:
        IndividualTracking(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh), _it(*_nh)
        {
            // get camera config from the parameter server
            _camera_cfg = top::get_camera_config(*nh);

            // set opencv stream
            std::string medium;

            if (_camera_cfg.using_file) {
                _camera = cv::VideoCapture(_camera_cfg.camera_dev_no);
                medium = "Using video file: " + _camera_cfg.camera_dev_no;
            }
            else {
                _camera = cv::VideoCapture(std::stoi(_camera_cfg.camera_dev_no), cv::CAP_GSTREAMER);
                medium = "Using camera device: " + _camera_cfg.camera_dev_no;
            }
            _camera.set(cv::CAP_PROP_FRAME_WIDTH, _camera_cfg.camera_px_width);
            _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _camera_cfg.camera_px_height);

            ROS_INFO("%s", medium.c_str());
            if (!_camera.isOpened()) {
                ROS_ERROR("ERROR: Could not open camera/file");
                assert(false);
            }

            // publisher for the image_transport wrapped image
            _raw_image_pub = _it.advertise("top_camera/image_raw", 1);
            _undistorted_image_pub = _it.advertise("top_camera/image_undistorted", 1);
            _masked_image_pub = _it.advertise("top_camera/image_masked", 1);
            _blob_pub = _it.advertise("top_camera/image_blobs", 1);

            // pose publisher
            _pose_pub = nh->advertise<bobi_msgs::PoseVec>("naive_poses", 1);

            _bd = std::make_unique<BlobDetector>(_cfg, true);

            _distortion_coeffs = cv::Mat(1, 5, CV_64F);
            memcpy(_distortion_coeffs.data, _camera_cfg.distortion_coeffs.data(), _camera_cfg.distortion_coeffs.size() * sizeof(double));
            _camera_mat = cv::Mat(3, 3, CV_64F);
            memcpy(_camera_mat.data, _camera_cfg.camera_matrix.data(), _camera_cfg.camera_matrix.size() * sizeof(double));

            _new_camera_mat = cv::getOptimalNewCameraMatrix(_camera_mat, _distortion_coeffs, cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), 1., cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), &_roi);
            ROS_WARN("Undistorted image ROI set to: %d x %d", _roi.size().width, _roi.size().height);

            _setup_mask = bobi::MaskFactory()(_camera_cfg.mask_type, _camera_cfg.mask_specs, cv::Size(_camera_cfg.camera_px_width, _camera_cfg.camera_px_height), CV_8UC3);
        }

        bool read()
        {
            return _camera.read(_frame);
        }

        void process()
        {
            cv::cvtColor(_frame, _frame, cv::COLOR_BGR2GRAY); // enforce image_transport compatible grayscale

            cv::undistort(_frame, _frame_und, _camera_mat, _distortion_coeffs, _new_camera_mat);
            _frame_und = _frame_und(_roi);

            _masked_frame = _frame_und.clone();
            _setup_mask->roi(_masked_frame);

            // detect individuals
            std::tie(_poses2d, _contours) = _bd->detect(_masked_frame);

            std::vector<int> idcs_to_remove;
            for (size_t i = 0; i < _poses2d.size(); ++i) {
                cv::patchNaNs(_poses2d, -1);
                if (_poses2d[i].x < 0 || _poses2d[i].y < 0) {
                    idcs_to_remove.push_back(i);
                }
            }

            for (size_t i = 0; i < idcs_to_remove.size(); ++i) {
                auto it = _poses2d.begin();
                std::advance(it, idcs_to_remove[i] - i);
                _poses2d.erase(it);
            }
        }

        void publish(std_msgs::Header header)
        {
            // publish the poses of the individuals that were detected
            bobi_msgs::PoseVec pv;
            for (size_t i = 0; i < _poses2d.size(); ++i) {
                const cv::Point3f& pose2d = _poses2d[i];
                bobi_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.xyz.x = pose2d.x * _camera_cfg.pix2m;
                pose.pose.xyz.y = pose2d.y * _camera_cfg.pix2m;
                pose.pose.rpy.yaw = pose2d.z;
                for (const cv::Point point : _contours[i]) {
                    geometry_msgs::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    pose.pose.contours.push_back(p);
                }
                pv.poses.push_back(pose);
            }
            _pose_pub.publish(pv);

            // publish the image that contains the filtered image
            sensor_msgs::ImagePtr blob_image_ptr = cv_bridge::CvImage(header, "mono8", _bd->get_blob_frame()).toImageMsg();
            _blob_pub.publish(blob_image_ptr);

            // publish raw image
            sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "mono8", _frame).toImageMsg();
            _raw_image_pub.publish(raw_image_ptr);

            sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "mono8", _frame_und).toImageMsg();
            _undistorted_image_pub.publish(undistorted_image_ptr);

            sensor_msgs::ImagePtr masked_image_ptr = cv_bridge::CvImage(header, "mono8", _masked_frame).toImageMsg();
            _masked_image_pub.publish(masked_image_ptr);
        }

    protected:
        std::shared_ptr<ros::NodeHandle> _nh;
        image_transport::ImageTransport _it;

        top::CameraConfig _camera_cfg;
        cv::VideoCapture _camera;

        // publisher for the image_transport wrapped image
        image_transport::Publisher _raw_image_pub;
        image_transport::Publisher _undistorted_image_pub;
        image_transport::Publisher _masked_image_pub;
        image_transport::Publisher _blob_pub;

        // pose publisher
        ros::Publisher _pose_pub;

        defaults::BlobDetectorConfig _cfg;
        std::unique_ptr<BlobDetector> _bd;

        cv::Mat _distortion_coeffs;
        cv::Mat _camera_mat;
        cv::Mat _new_camera_mat;

        cv::Rect _roi;
        bobi::MaskPtr _setup_mask;

        cv::Mat _frame;
        cv::Mat _frame_und;
        cv::Mat _masked_frame;

        std::vector<cv::Point3f> _poses2d;
        std::vector<std::vector<cv::Point>> _contours;
    };
} // namespace bobi

#endif