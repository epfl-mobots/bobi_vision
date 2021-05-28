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
            _poses.resize(_led_colours.size());
        }

        std::vector<cv::Point3f> detect(cv::Mat& frame)
        {
            typedef std::vector<std::vector<cv::Point>> Contours;
            std::vector<Contours> all_contours;
            std::vector<cv::Point3f> new_poses;
            for (size_t idx = 0; idx < _led_colours.size(); ++idx) {
                const cv::Scalar& led = _led_colours[idx];

                cv::Mat blurred;
                cv::blur(frame, blurred, cv::Size(4, 4));

                cv::Mat hsv;
                cv::cvtColor(blurred, hsv, cv::COLOR_RGB2HSV);

                cv::Mat binary;
                cv::inRange(hsv,
                    cv::Scalar(led[0] * _hsv_low[0], led[1] * _hsv_low[1], led[2] * _hsv_low[2]),
                    cv::Scalar(led[0] * _hsv_high[0], led[1] * _hsv_high[1], 255 * _hsv_high[2]),
                    binary); //

                cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1)));
                cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1)));
                cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1)));
                cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1)));

                Contours contours;
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                std::vector<int> idcs(contours.size());
                std::iota(idcs.begin(), idcs.end(), 0);
                std::sort(idcs.begin(), idcs.end(), [&contours](int lhs, int rhs) {
                    return cv::contourArea(contours[lhs], false) > cv::contourArea(contours[rhs], false);
                });

                cv::imshow("binary", binary);
                cv::waitKey(10);

                if (contours.size() != 2) {
                    std::cout << contours.size() << std::endl;
                }

                cv::Point3f pose = _poses[idx];
                if (contours.size() >= 2) {
                    cv::Point2f cnt0 = _contour_center(contours[0]);
                    cv::Point2f cnt1 = _contour_center(contours[1]);

                    if (_prev_contours.size() == 0) {
                        _prev_contours.resize(2);
                        _prev_contours[0] = cnt0;
                        _prev_contours[1] = cnt1;
                    }
                    else {
                        std::vector<cv::Point2f> new_contours = {cnt0, cnt1};
                        std::sort(new_contours.begin(), new_contours.end(),
                            [&](const cv::Point2f& x, const cv::Point2f& y) {
                                return cv::norm(x - _prev_contours[0]) < cv::norm(y - _prev_contours[0]);
                            });
                        cnt0 = new_contours[0];
                        cnt1 = new_contours[1];
                    }

                    pose.x = (cnt0.x + cnt1.x) / 2.;
                    pose.y = (cnt0.y + cnt1.y) / 2.;
                    pose.z = std::atan2((cnt1.y - cnt0.y), cnt1.x - cnt0.x);

                    cv::Point2f vector = (cnt1 + _prev_contours[1] / 2) - (cnt0 + _prev_contours[0] / 2);
                    cv::Point2f displacement = cv::Point2f(pose.x, pose.y) - cv::Point2f(_poses[idx].x, _poses[idx].y);

                    if (vector.dot(displacement) < 0. && cv::norm(displacement) > 0.55) {
                        pose.z = _angle_to_pipi(pose.z + M_PI);
                        _prev_contours = {cnt1, cnt0};
                    }
                    else {
                        _prev_contours = {cnt0, cnt1};
                    }
                }

                new_poses.push_back(pose);
                all_contours.push_back(contours);
            }

            for (const auto& led_contours : all_contours) {
                for (auto& contour : led_contours) {
                    cv::circle(frame, _contour_center(contour), 2, cv::Scalar(0, 255, 0));
                }
            }

            _poses = new_poses;
            return _poses;
        }

    protected:
        inline cv::Point2f _contour_center(const std::vector<cv::Point> contour) const
        {
            cv::Moments m = cv::moments(contour, true);
            return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        }

        double _angle_to_pipi(double angle)
        {
            while (true) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
                if (abs(angle) <= M_PI) {
                    break;
                }
            }
            return angle;
        }

        template <typename T>
        int _sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        std::vector<cv::Scalar> _led_colours;
        std::vector<cv::Point3f> _poses;
        std::vector<cv::Point2f> _prev_contours;

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