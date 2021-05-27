#ifndef BOBI_COLOUR_DETECTOR_H
#define BOBI_COLOUR_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <bobi_vision/ColourDetectorConfig.h>

#include <numeric>

namespace bobi {
    namespace defaults {
        struct ColourDetectorConfig {
        };
    } // namespace defaults

    class ColourDetector {
    public:
        ColourDetector(const std::vector<cv::Scalar>& led_colours)
            : _led_colours(led_colours)
        {
            dynamic_reconfigure::Server<bobi_vision::ColourDetectorConfig>::CallbackType f;
            f = boost::bind(&ColourDetector::_config_cb, this, _1, _2);
            _config_server.setCallback(f);
        }

        std::vector<cv::Point3f> detect(cv::Mat& frame)
        {
            for (const cv::Scalar& led : _led_colours) {
                cv::Mat blurred;
                cv::blur(frame, blurred, cv::Size(3, 3));

                cv::Mat hsv;
                cv::cvtColor(blurred, hsv, cv::COLOR_RGB2HSV);

                cv::Mat binary;
                cv::inRange(hsv,
                    cv::Scalar(led[0] * _hsv_low[0], led[1] * _hsv_low[1], led[2] * _hsv_low[2]),
                    cv::Scalar(led[0] * _hsv_high[0], led[1] * _hsv_high[1], 255 * _hsv_high[2]),
                    binary); //

                cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));
                cv::erode(binary, binary, element);
                cv::dilate(binary, binary, element);
                cv::dilate(binary, binary, element);
                cv::erode(binary, binary, element);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                std::vector<int> idcs(contours.size());
                std::iota(idcs.begin(), idcs.end(), 0);
                std::sort(idcs.begin(), idcs.end(), [&contours](int lhs, int rhs) {
                    return cv::contourArea(contours[lhs], false) > cv::contourArea(contours[rhs], false);
                });

                for (auto& contour : contours) {
                    cv::circle(blurred, _contour_center(contour), 2, cv::Scalar(0, 255, 0));
                }

                cv::imshow("debug", blurred);
                cv::imshow("hsv", hsv);
                cv::imshow("masked", binary);
                cv::waitKey(10);
            }

            return std::vector<cv::Point3f>();
        }

    protected:
        inline cv::Point2f _contour_center(const std::vector<cv::Point> contour) const
        {
            cv::Moments m = cv::moments(contour, true);
            return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        }

        std::vector<cv::Scalar> _led_colours;

    private:
        void _config_cb(bobi_vision::ColourDetectorConfig& config, uint32_t level)
        {
            ROS_INFO("Updated config");
            _hsv_low = {config.h_low, config.s_low, config.v_low};
            _hsv_high = {config.h_high, config.s_high, config.v_high};
        }

        std::vector<double> _hsv_low;
        std::vector<double> _hsv_high;

        dynamic_reconfigure::Server<bobi_vision::ColourDetectorConfig> _config_server;
    };

} // namespace bobi

#endif