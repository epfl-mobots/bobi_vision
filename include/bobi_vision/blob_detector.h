#ifndef BOBI_BLOB_DETECTOR_H
#define BOBI_BLOB_DETECTOR_H

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <bobi_vision/BlobDetectorConfig.h>
namespace bobi {
    namespace defaults {
        struct BlobDetectorConfig {
            // behavioural
            size_t num_agents = 5;

            // background subtractor
            size_t num_background_samples = 200;
            int bstor_history = 500;
            double var_threshold = 4;
            bool detect_shadows = false;
            float learning_rate = 0.05;
            size_t min_contour_size = 0;

            // tracking features
            double quality_level = 0.01;
            double min_distance = 5.;
            int block_size = 5;
            bool use_harris_dtor = true;
            double k = 0.04;

            // Binary threshold
            int threshold_new_value = 80;
            int threhold_value = 150;
        };
    } // namespace defaults

    class BlobDetector {
    public:
        BlobDetector(defaults::BlobDetectorConfig config = defaults::BlobDetectorConfig(), bool annotate_frame = false);

        std::vector<cv::Point3f> detect(cv::Mat& frame);
        void reset_background_detector();

        cv::Mat get_blob_frame() const;

    protected:
        void _remove_contours(cv::Mat& frame);
        void _find_contours(cv::Mat& image,
            cv::Mat& annot_frame,
            const std::vector<cv::Point2f>& corners,
            std::vector<cv::Point2f>& centers,
            std::vector<std::vector<cv::Point2f>>& corners_in_contours) const;
        cv::Point2f _contour_center(const std::vector<cv::Point> contour) const;
        std::vector<cv::Point3f> _naive_pose2d(const std::vector<cv::Point2f>& centers,
            const std::vector<std::vector<cv::Point2f>>& corners_in_contours) const;

        cv::Mat _gray_frame;
        cv::Mat _foreground_frame;
        cv::Mat _blob_frame;
        bool _annotate_frame;
        defaults::BlobDetectorConfig _config;

    private:
        void blobDetectorConfigCallback(bobi_vision::BlobDetectorConfig& config, uint32_t level);

        dynamic_reconfigure::Server<bobi_vision::BlobDetectorConfig> _bd_config_server;
        cv::Ptr<cv::BackgroundSubtractor> _background_stor;
        size_t _background_counter;
    };

} // namespace bobi

#endif