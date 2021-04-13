#ifndef BOBI_FRAME_INFO_HPP
#define BOBI_FRAME_INFO_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace bobi {
    class FrameInfo {
    public:
        FrameInfo()
        {
            _prev_time = ros::Time::now();
        }

        void draw_all(cv::Mat& frame)
        {
            draw_fps(frame);
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

    protected:
        ros::Time _prev_time;
    };
} // namespace bobi

#endif