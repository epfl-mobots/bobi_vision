#ifndef BOBI_COLOUR_DETECTOR_H
#define BOBI_COLOUR_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

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
        }

        std::vector<cv::Point3f> detect(cv::Mat& frame)
        {
            for (const cv::Scalar& led : _led_colours) {
                cv::Scalar hsv_led = _bgr2hsv(led);

                cv::Mat blurred;
                cv::blur(frame, blurred, cv::Size(3, 3));

                cv::Mat hsv;
                cv::cvtColor(blurred, hsv, cv::COLOR_RGB2HSV);

                cv::Mat binary;
                cv::inRange(hsv,
                    cv::Scalar(hsv_led[0] * 0.2, 0, 0),
                    cv::Scalar(hsv_led[0] * 1.6, 255, 255),
                    binary); //

                cv::imshow("debug", frame);
                cv::imshow("hsv", hsv);
                cv::imshow("masked", binary);
                cv::waitKey(10);
            }

            return std::vector<cv::Point3f>();

            //         // convert to hsv
            //         cv::cvtColor(m_blurredImage, m_hsvImage, CV_RGB2HSV);
            //         // threshold the image in the HSV color space
            //         cv::inRange(m_hsvImage,
            //                     cv::Scalar(h / 2 - tolerance, 0 , v - 2 * tolerance),
            //                     cv::Scalar(h / 2 + tolerance, 255, 255),
            //                     m_binaryImage); //

            //         // postprocessing of the binary image
            //         int an = 1;
            //         cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(an*2+1, an*2+1), cv::Point(an, an)); // TODO : inititialize this in the constructor

            //         //morphological opening (remove small objects from the foreground)
            //         cv::erode(m_binaryImage, m_binaryImage, element);
            //         cv::dilate(m_binaryImage, m_binaryImage, element);

            //         //morphological closing (fill small holes in the foreground)
            //         cv::dilate(m_binaryImage, m_binaryImage, element);
            //         cv::erode(m_binaryImage, m_binaryImage, element);

            // //        // submit the debug image
            // //        if (m_enqueueDebugFrames) {
            // //            enqueueDebugImage(m_binaryImage);
            // //        }
            //         // TODO : make a devoted method
            //         // detect the spots as contours
            //         std::vector<std::vector<cv::Point>> contours;
            //         try { // TODO : to check if this try-catch can be removed or if it should be used everywhere where opencv methods are used.
            //             // retrieve contours from the binary image
            //             cv::findContours(m_binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            //         } catch (const cv::Exception& e) {
            //             qDebug() << "OpenCV exception: " << e.what();
            //         }

            //         // sort the contours by size (inspired by http://stackoverflow.com/questions/33401745/find-largest-contours-opencv)
            //         std::vector<int> indices(contours.size());
            //         std::iota(indices.begin(), indices.end(), 0);
            //         std::sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
            //             return cv::contourArea(contours[lhs],false) > cv::contourArea(contours[rhs],false);
            //         });

            //         // centers of contour
            //         int agentIndex = 0;
            //         for (auto& contour: contours) {
            //             if (agentIndex < m_agents.size()) {
            //                 m_agents[agentIndex].mutableState()->setPosition(contourCenter(contour));
            //                 agentIndex++;
            //             } else {
            //                 break;
            //             }
            //         }

            //         // submit the debug image
            //         if (m_enqueueDebugFrames) {
            //             for (auto& agent: m_agents) {
            //                 cv::circle(m_blurredImage, cv::Point(agent.state().position().x(), agent.state().position().y()), 2, cv::Scalar(255, 255, 255));
            //             }
            //             enqueueDebugImage(m_blurredImage);
            //         }
            //     }
            //     else
            //         qDebug() << "Unsupported image format" << image.type();
        }

    protected:
        inline cv::Scalar _bgr2hsv(const cv::Scalar& bgr)
        {
            cv::Mat hsv;
            cvtColor(cv::Mat(1, 1, CV_8UC3, bgr), hsv, cv::COLOR_BGR2HSV);
            return cv::Scalar(hsv.at<uint8_t>(0, 0), hsv.at<uint8_t>(0, 1), hsv.at<uint8_t>(0, 2));
        }

        std::vector<cv::Scalar> _led_colours;

    private:
        dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig> _bd_config_server;
    };

} // namespace bobi

#endif