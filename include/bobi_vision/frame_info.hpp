#ifndef BOBI_FRAME_INFO_HPP
#define BOBI_FRAME_INFO_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/ConvertCoordinates.h>
#include <bobi_vision/BlobDetectorConfig.h>
#include <bobi_vision/mask_factory.hpp>

#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>

namespace bobi {
    enum CameraLocation {
        NA,
        TOP,
        BOTTOM
    };

    class FrameInfo {
    public:
        FrameInfo(std::shared_ptr<ros::NodeHandle> nh)
            : _nh(nh),
              _top_fps(0.),
              _bottom_fps(0.)
        {
            _prev_time = ros::Time::now();

            _bottom2top_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_bottom2top");
            _top2bottom_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_top2bottom");
            _bottom2top_srv.waitForExistence();
            _top2bottom_srv.waitForExistence();

            dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig>::CallbackType f;
            f = boost::bind(&FrameInfo::_config_cb, this, _1, _2);
            _config_server.setCallback(f);

            _target_position = _nh->advertise<bobi_msgs::PoseStamped>("target_position", 1);
        }

        void draw_all(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> individual_poses,
            std::vector<bobi_msgs::PoseStamped> robot_poses,
            bobi_msgs::PoseStamped target_position,
            bool ifiltered,
            bool rfiltered,
            CameraLocation camera_loc = CameraLocation::NA)
        {
            draw_fps(frame);
            draw_poses(frame, individual_poses, camera_loc, ifiltered, rfiltered);
            draw_robot_poses(frame, robot_poses, camera_loc, ifiltered, rfiltered);
            draw_center(frame, camera_loc);
            draw_mask(frame, camera_loc);
            draw_target(frame, target_position, camera_loc);
        }

        void draw_fps(cv::Mat& frame)
        {
            ros::Time now = ros::Time::now();
            double dt = ros::Duration(now - _prev_time).toSec();

            {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << 1. / dt;
                cv::putText(frame,
                    "FPS (processing): " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 0, 0),
                    2);
            }

            {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << _top_fps;
                cv::putText(frame,
                    "FPS (top): " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.09),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 0, 0),
                    2);
            }

            {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << _bottom_fps;
                cv::putText(frame,
                    "FPS (bottom): " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.13),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(200, 0, 0),
                    2);
            }

            _prev_time = now;
        }

        void draw_poses(cv::Mat& frame, std::vector<bobi_msgs::PoseStamped> poses, const CameraLocation camera_loc, bool ifiltered, bool rfiltered)
        {
            if (poses.size() > 0) {
                double cur_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_top_header.stamp).toSec();
                if (cur_fps != std::numeric_limits<double>::infinity()) {
                    _top_fps = cur_fps;
                }
                _prev_top_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];
                if (camera_loc == CameraLocation::BOTTOM) {
                    bobi_msgs::ConvertCoordinates srv;
                    srv.request.p.x = pose.pose.xyz.x;
                    srv.request.p.y = pose.pose.xyz.y;
                    if (_top2bottom_srv.call(srv)) {
                        pose.pose.xyz.x = srv.response.converted_p.x / _bottom_pix2m;
                        pose.pose.xyz.y = srv.response.converted_p.y / _bottom_pix2m;
                    }
                }
                else {
                    pose.pose.xyz.x /= _top_pix2m;
                    pose.pose.xyz.y /= _top_pix2m;

                    {
                        std::stringstream stream;
                        stream << std::fixed << std::setprecision(1) << _top_fps;

                        std::string msg;
                        cv::Scalar colour;
                        if (ifiltered) {
                            msg = "Filtering: ACTIVE";
                            colour = CV_RGB(0, 200, 0);
                        }
                        else {
                            msg = "Filtering: INACTIVE";
                            colour = CV_RGB(200, 0, 0);
                        }
                        cv::putText(frame,
                            msg,
                            cv::Point(frame.size().width * 0.03, frame.size().height * 0.97),
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            colour,
                            2);
                    }
                }

                float offset = 13.;
                float base_coef = 4.;
                cv::Point p1(pose.pose.xyz.x + 2 * offset * std::cos(pose.pose.rpy.yaw), pose.pose.xyz.y + 2 * offset * std::sin(pose.pose.rpy.yaw));
                cv::Point p2(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw + M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw + M_PI / base_coef));
                cv::Point p3(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw - M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw - M_PI / base_coef));

                const cv::Point points[] = {p1, p2, p3};

                // cv::circle(frame, p1, 2, cv::Scalar(0, 0, 255));
                // cv::circle(frame, p2, 2, cv::Scalar(0, 255, 0));
                // cv::circle(frame, p3, 2, cv::Scalar(255, 0, 0));

                std::lock_guard<std::mutex> guard(_cfg_mutex);
                cv::line(frame, p1, p2, _colours[i], 2, cv::LINE_8);
                cv::line(frame, p2, p3, _colours[i], 2, cv::LINE_8);
                cv::line(frame, p3, p1, _colours[i], 2, cv::LINE_8);
            }
        }

        void draw_robot_poses(cv::Mat& frame, std::vector<bobi_msgs::PoseStamped> poses, const CameraLocation camera_loc, bool ifiltered, bool rfiltered)
        {
            if (poses.size() > 0) {
                double cur_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_bottom_header.stamp).toSec();
                if (cur_fps != std::numeric_limits<double>::infinity()) {
                    _bottom_fps = cur_fps;
                }
                _prev_bottom_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];

                if (camera_loc == CameraLocation::TOP) {
                    bobi_msgs::ConvertCoordinates srv;
                    srv.request.p.x = pose.pose.xyz.x;
                    srv.request.p.y = pose.pose.xyz.y;
                    if (_bottom2top_srv.call(srv)) {
                        pose.pose.xyz.x = srv.response.converted_p.x / _top_pix2m;
                        pose.pose.xyz.y = srv.response.converted_p.y / _top_pix2m;
                    }
                }
                else {
                    pose.pose.xyz.x /= _bottom_pix2m;
                    pose.pose.xyz.y /= _bottom_pix2m;
                }

                float offset = 13.;
                float base_coef = 4.;
                cv::Point p1(pose.pose.xyz.x + 2 * offset * std::cos(pose.pose.rpy.yaw), pose.pose.xyz.y + 2 * offset * std::sin(pose.pose.rpy.yaw));
                cv::Point p2(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw + M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw + M_PI / base_coef));
                cv::Point p3(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw - M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw - M_PI / base_coef));

                const cv::Point points[] = {p1, p2, p3};

                // cv::circle(frame, p1, 2, cv::Scalar(0, 0, 255));
                // cv::circle(frame, p2, 2, cv::Scalar(0, 255, 0));
                // cv::circle(frame, p3, 2, cv::Scalar(255, 0, 0));

                std::lock_guard<std::mutex> guard(_cfg_mutex);
                cv::line(frame, p1, p2, _colours[i], 2, cv::LINE_8);
                cv::line(frame, p2, p3, _colours[i], 2, cv::LINE_8);
                cv::line(frame, p3, p1, _colours[i], 2, cv::LINE_8);
            }
        }

        void draw_center(cv::Mat& frame, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::TOP:
                cv::circle(frame, cv::Point(frame.size().width / 2., frame.size().height / 2.), 2, cv::Scalar(0, 255, 0), cv::FILLED);
                break;
            case CameraLocation::BOTTOM:
            default:
                break;
            }
        }

        void draw_mask(cv::Mat& frame, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::TOP:
                _top_mask->draw_roi(frame, cv::Scalar(0, 255, 0));
                break;

            case CameraLocation::BOTTOM:
                _bottom_mask->draw_roi(frame, cv::Scalar(0, 255, 0));
                break;
            }
        }

        void draw_target(cv::Mat& frame, bobi_msgs::PoseStamped target, const CameraLocation camera_loc)
        {
            switch (camera_loc) {
            case CameraLocation::BOTTOM:
                if (target.pose.xyz.x >= 0 && target.pose.xyz.y >= 0) {
                    cv::drawMarker(frame, cv::Point(target.pose.xyz.x / _bottom_pix2m, target.pose.xyz.y / _bottom_pix2m), cv::Scalar(0, 200, 0), cv::MARKER_TILTED_CROSS, 15, 2);
                }
                break;

            case CameraLocation::TOP:
                if (target.pose.xyz.x >= 0 && target.pose.xyz.y >= 0) {
                    bobi_msgs::ConvertCoordinates srv;
                    srv.request.p.x = target.pose.xyz.x;
                    srv.request.p.y = target.pose.xyz.y;
                    if (_bottom2top_srv.call(srv)) {
                        target.pose.xyz.x = srv.response.converted_p.x / _top_pix2m;
                        target.pose.xyz.y = srv.response.converted_p.y / _top_pix2m;
                        cv::drawMarker(frame, cv::Point(target.pose.xyz.x, target.pose.xyz.y), cv::Scalar(0, 200, 0), cv::MARKER_TILTED_CROSS, 15, 2);
                    }
                }
                break;
            }
        }

        void set_top_pix2m(double coeff)
        {
            _top_pix2m = coeff;
        }

        void set_bottom_pix2m(double coeff)
        {
            _bottom_pix2m = coeff;
        }

        double get_top_pix2m() const
        {
            return _top_pix2m;
        }

        double get_bottom_pix2m() const
        {
            return _bottom_pix2m;
        }

        geometry_msgs::Point convert_top2bottom(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_top2bottom_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        geometry_msgs::Point convert_bottom2top(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_bottom2top_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        void set_top_mouse_coords(int x, int y)
        {
            std::lock_guard<std::mutex> guard(_topw_mutex);
            _top_mouse_coords.first.x = x;
            _top_mouse_coords.first.y = y;
            _top_mouse_coords.second.x = x * _top_pix2m;
            _top_mouse_coords.second.y = y * _top_pix2m;
        }

        void set_bottom_mouse_coords(int x, int y)
        {
            std::lock_guard<std::mutex> guard(_bottomw_mutex);
            _bottom_mouse_coords.first.x = x;
            _bottom_mouse_coords.first.y = y;
            _bottom_mouse_coords.second.x = x * _bottom_pix2m;
            _bottom_mouse_coords.second.y = y * _bottom_pix2m;
        }

        void set_target(const bobi_msgs::PoseStamped& ps)
        {
            _target_position.publish(ps);
        }

        void set_top_mask(bobi::MaskPtr mask)
        {
            _top_mask = mask;
        }

        void set_bottom_mask(bobi::MaskPtr mask)
        {
            _bottom_mask = mask;
        }

    protected:
        void _config_cb(bobi_vision::BlobDetectorConfig& config, uint32_t level)
        {
            _num_agents = config.num_agents;

            std::lock_guard<std::mutex> guard(_cfg_mutex);

            auto rand_in_range = [](int lb = 0, int ub = 255) { return (std::rand() % (ub - lb) + lb); };
            _colours.clear();
            for (size_t i = 0; i < _num_agents; ++i) {
                _colours.push_back(cv::Scalar(rand_in_range(40), rand_in_range(40), rand_in_range(40)));
            }
        }

        static void _top_cam_mouse_cb(int event, int x, int y, int flags, void* data)
        {
            FrameInfo* fi = static_cast<FrameInfo*>(data);
            if (event == cv::EVENT_MOUSEMOVE) {
                fi->set_top_mouse_coords(x, y);
            }

            if (flags == (cv::EVENT_FLAG_CTRLKEY + cv::EVENT_LBUTTONDOWN)) {
                geometry_msgs::Point p, converted_p;
                p.x = x * fi->get_top_pix2m();
                p.y = y * fi->get_top_pix2m();
                converted_p = fi->convert_top2bottom(p);
                bobi_msgs::PoseStamped ps;
                ps.pose.xyz.x = converted_p.x;
                ps.pose.xyz.y = converted_p.y;
                ps.header.stamp = ros::Time::now();
                fi->set_target(ps);
            }
        }

        static void _bottom_cam_mouse_cb(int event, int x, int y, int flags, void* data)
        {
            FrameInfo* fi = static_cast<FrameInfo*>(data);
            if (event == cv::EVENT_MOUSEMOVE) {
                fi->set_bottom_mouse_coords(x, y);
            }

            if (flags == (cv::EVENT_FLAG_CTRLKEY + cv::EVENT_LBUTTONDOWN)) {
                float xm = x * fi->get_bottom_pix2m();
                float ym = y * fi->get_bottom_pix2m();
                bobi_msgs::PoseStamped ps;
                ps.pose.xyz.x = xm;
                ps.pose.xyz.y = ym;
                ps.header.stamp = ros::Time::now();
                fi->set_target(ps);
            }
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::ServiceClient _top2bottom_srv;
        ros::ServiceClient _bottom2top_srv;
        ros::Publisher _target_position;

        dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig> _config_server;

        ros::Time _prev_time;
        size_t _num_agents;

        std::vector<cv::Scalar> _colours;
        std::mutex _cfg_mutex;

        double _top_fps;
        double _bottom_fps;
        std_msgs::Header _prev_top_header;
        std_msgs::Header _prev_bottom_header;

        std::mutex _topw_mutex;
        std::mutex _bottomw_mutex;
        std::pair<cv::Point, cv::Point2f> _top_mouse_coords;
        std::pair<cv::Point, cv::Point2f> _bottom_mouse_coords;

        double _top_pix2m;
        double _bottom_pix2m;

        bobi::MaskPtr _top_mask;
        bobi::MaskPtr _bottom_mask;
    };
} // namespace bobi

#endif