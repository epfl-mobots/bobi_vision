#ifndef BOBI_COLOUR_DETECTOR_H
#define BOBI_COLOUR_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <numeric>

namespace bobi {
    namespace defaults {
        struct ColourDetectorConfig {
        };
    } // namespace defaults

    typedef std::vector<std::array<cv::Scalar, 2>> LedPairVec;
    typedef std::tuple<double, double, double, double, double, double> Tuple7d;
    typedef std::vector<std::array<Tuple7d, 2>> TuplePairVec;
    typedef std::vector<std::vector<cv::Point>> Contours;

    class ColourDetector {
    public:
        ColourDetector(const LedPairVec& led_colours, const TuplePairVec& hsv_thresholds)
            : _led_colours(led_colours),
              _hsv_thresholds(hsv_thresholds)
        {
            _poses.resize(_led_colours.size());
        }

        std::vector<cv::Point3f> detect(cv::Mat& frame)
        {
            cv::Mat blurred;
            cv::blur(frame, blurred, cv::Size(4, 4));

            cv::Mat hsv;
            cv::cvtColor(blurred, hsv, cv::COLOR_RGB2HSV);

            cv::Mat binary;
            std::vector<Contours> all_contours;
            std::vector<cv::Point3f> new_poses;
            for (size_t idx = 0; idx < _led_colours.size(); ++idx) {

                std::vector<cv::Point> new_contours;
                for (size_t k = 0; k < 2; ++k) {
                    const cv::Scalar& led = _led_colours[idx][k];
                    const Tuple7d& thres = _hsv_thresholds[idx][k];

                    cv::inRange(hsv.clone(),
                        cv::Scalar(
                            led[0] * std::get<0>(thres), led[1] * std::get<1>(thres), led[2] * std::get<2>(thres)),
                        cv::Scalar(
                            led[0] * std::get<3>(thres), led[1] * std::get<4>(thres), 255 * std::get<5>(thres)),
                        binary);

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

                    if (contours.size() >= 1) {
                        new_contours.push_back(_contour_center(contours[0]));
                    }
                }

                cv::Point3f pose = _poses[idx];
                if (new_contours.size() == 2) {
                    pose.x = (new_contours[0].x + new_contours[1].x) / 2.;
                    pose.y = (new_contours[0].y + new_contours[1].y) / 2.;
                    pose.z = std::atan2((new_contours[0].y - new_contours[1].y), new_contours[0].x - new_contours[1].x);
                }
                new_poses.push_back(pose);
                all_contours.push_back({new_contours});
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
        inline cv::Point _contour_center(const std::vector<cv::Point> contour) const
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

        LedPairVec _led_colours;
        TuplePairVec _hsv_thresholds;
        std::vector<cv::Point3f> _poses;

    private:
        std::vector<double> _hsv_low;
        std::vector<double> _hsv_high;

    }; // namespace bobi

} // namespace bobi

#endif