#ifndef BOBI_FRAME_INFO_HPP
#define BOBI_FRAME_INFO_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <bobi_msgs/PoseStamped.h>

namespace bobi {
    class FrameInfo {
    public:
        FrameInfo()
        {
            _prev_time = ros::Time::now();
        }

        void draw_all(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> individual_poses,
            std::vector<bobi_msgs::PoseStamped> robot_poses)
        {
            draw_fps(frame);
            draw_poses(frame, individual_poses);
        }

        void draw_fps(cv::Mat& frame)
        {
            ros::Time now = ros::Time::now();
            double dt = ros::Duration(now - _prev_time).toSec();

            std::stringstream stream;
            stream << std::fixed << std::setprecision(1) << 1 / dt;
            cv::putText(frame,
                "FPS: " + stream.str(),
                cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(0, 0, 0),
                2);

            _prev_time = now;
        }

        void draw_poses(
            cv::Mat& frame,
            std::vector<bobi_msgs::PoseStamped> poses)
        {
            for (const bobi_msgs::PoseStamped pose : poses) {
                float offset = 13.;
                float base_coef = 4.;
                cv::Point p1(pose.pose.xyz.x + 2 * offset * std::cos(pose.pose.rpy.yaw), pose.pose.xyz.y + 2 * offset * std::sin(pose.pose.rpy.yaw));
                cv::Point p2(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw + M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw + M_PI / base_coef));
                cv::Point p3(pose.pose.xyz.x - 1.5 * offset * std::cos(pose.pose.rpy.yaw - M_PI / base_coef), pose.pose.xyz.y - 1.5 * offset * std::sin(pose.pose.rpy.yaw - M_PI / base_coef));

                const cv::Point points[] = {p1, p2, p3};

                // cv::circle(frame, p1, 2, cv::Scalar(0, 0, 255));
                // cv::circle(frame, p2, 2, cv::Scalar(0, 255, 0));
                // cv::circle(frame, p3, 2, cv::Scalar(255, 0, 0));

                // TODO: give unique colours to each agent
                cv::line(frame, p1, p2, cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                cv::line(frame, p2, p3, cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                cv::line(frame, p3, p1, cv::Scalar(0, 0, 0), 2, cv::LINE_8);
            }
        }

    protected:
        ros::Time _prev_time;
    };
} // namespace bobi

#endif