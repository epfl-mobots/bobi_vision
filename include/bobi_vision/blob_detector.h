#ifndef BOBI_BLOB_DETECTOR_H
#define BOBI_BLOB_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace bobi {
    namespace defaults {
        struct BlobDetectorConfig {
            // behavioural
            size_t num_agents = 2;

            // background subtractor
            size_t num_background_samples = 100;
            int bstor_history = 500;
            double var_threshold = 5;
            bool detect_shadows = false;
            float learning_rate = 0.05;

            // tracking features
            double quality_level = 0.01;
            double min_distance = 5.;
            int block_size = 5;
            bool use_harris_dtor = true;
            double k = 0.04;
        };
    } // namespace defaults

    class BlobDetector {
    public:
        // TODO: set up dynamic reconfigure
        BlobDetector(defaults::BlobDetectorConfig config = defaults::BlobDetectorConfig(), bool annotate_frame = false);

        std::vector<cv::Point3f> detect(cv::Mat& frame);
        void reset_background_detector();

    protected:
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
        bool _annotate_frame;
        defaults::BlobDetectorConfig _config;

    private:
        cv::Ptr<cv::BackgroundSubtractor> _background_stor;
        size_t _background_counter;
    };

} // namespace bobi

#endif