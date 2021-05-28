#ifndef BOBI_FRAME_INFO_HPP
#define BOBI_FRAME_INFO_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <bobi_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>
#include <bobi_vision/BlobDetectorConfig.h>

#include <std_msgs/Header.h>

namespace bobi {
    class FrameInfo {
    public:
        FrameInfo()
        {
            _prev_time = ros::Time::now();

            dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig>::CallbackType f;
            f = boost::bind(&FrameInfo::_config_cb, this, _1, _2);
            _config_server.setCallback(f);
        }

        void draw_all(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> individual_poses,
            std::vector<bobi_msgs::PoseStamped> robot_poses)
        {
            draw_fps(frame);
            draw_poses(frame, individual_poses);
            draw_robot_poses(frame, robot_poses);
        }

        void draw_fps(cv::Mat& frame)
        {
            ros::Time now = ros::Time::now();
            double dt = ros::Duration(now - _prev_time).toSec();

            {
                std::stringstream stream;
                stream << std::fixed << std::setprecision(1) << 1 / dt;
                cv::putText(frame,
                    "FPS (processing): " + stream.str(),
                    cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(0, 0, 0),
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
                    CV_RGB(0, 0, 0),
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
                    CV_RGB(0, 0, 0),
                    2);
            }

            _prev_time = now;
        }

        void draw_poses(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> poses)
        {
            if (poses.size() > 0) {
                _top_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_header.stamp).toSec();
                _prev_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];
                pose.pose.xyz.x /= _top_pix2m;
                pose.pose.xyz.y /= _top_pix2m;

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

        void draw_robot_poses(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> poses)
        {
            if (poses.size() > 0) {
                _bottom_fps = 1. / ros::Duration(poses[0].header.stamp - _prev_header.stamp).toSec();
                _prev_header = poses[0].header;
            }

            for (size_t i = 0; i < poses.size(); ++i) {
                bobi_msgs::PoseStamped pose = poses[i];
                pose.pose.xyz.x /= _bottom_pix2m;
                pose.pose.xyz.y /= _bottom_pix2m;

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

        void set_top_pix2m(double coeff)
        {
            _top_pix2m = coeff;
        }

        void set_bottom_pix2m(double coeff)
        {
            _bottom_pix2m = coeff;
        }

    protected:
        void _config_cb(bobi_vision::BlobDetectorConfig& config, uint32_t level)
        {
            _num_agents = config.num_agents;
            _num_robots = config.num_robots;

            std::lock_guard<std::mutex> guard(_cfg_mutex);

            auto rand_in_range = [](int lb = 0, int ub = 255) { return (std::rand() % (ub - lb) + lb); };
            _colours.clear();
            for (size_t i = 0; i < _num_agents; ++i) {
                _colours.push_back(cv::Scalar(rand_in_range(40), rand_in_range(40), rand_in_range(40)));
            }
        }

        dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig> _config_server;

        ros::Time _prev_time;
        size_t _num_agents;
        size_t _num_robots;

        std::vector<cv::Scalar> _colours;
        std::mutex _cfg_mutex;

        double _top_fps;
        double _bottom_fps;
        std_msgs::Header _prev_header;

        double _top_pix2m;
        double _bottom_pix2m;
    };
} // namespace bobi

#endif