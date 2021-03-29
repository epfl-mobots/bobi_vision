#include <bobi_vision/blob_detector.h>
#include <iostream>

namespace bobi {

    BlobDetector::BlobDetector(defaults::BlobDetectorConfig config, bool annotate_frame)
        : _config(config),
          _background_stor(cv::createBackgroundSubtractorMOG2(_config.bstor_history, _config.var_threshold, _config.detect_shadows)),
          _background_counter(0),
          _annotate_frame(annotate_frame)
    {
    }

    std::vector<cv::Point3f> BlobDetector::detect(cv::Mat& frame)
    {
        cv::cvtColor(frame, _gray_frame, cv::COLOR_RGB2GRAY);

        if (_background_counter < _config.num_background_samples) {
            _background_stor.get()->apply(_gray_frame, _foreground_frame, _config.learning_rate);
            ++_background_counter;
            return std::vector<cv::Point3f>();
        }

        _background_stor.get()->apply(_gray_frame, _foreground_frame, 0.);
        cv::morphologyEx(_foreground_frame, _foreground_frame, cv::MORPH_DILATE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2), cv::Point(1, 1)));
        cv::morphologyEx(_foreground_frame, _foreground_frame, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2), cv::Point(1, 1)));
        cv::morphologyEx(_foreground_frame, _foreground_frame, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8), cv::Point(1, 1)));

        std::vector<cv::Point2f> corners;
        std::vector<cv::Point2f> centers;
        std::vector<std::vector<cv::Point2f>> corners_in_contours;
        try {
            cv::goodFeaturesToTrack(_gray_frame,
                corners,
                _config.num_agents,
                _config.quality_level,
                _config.min_distance,
                _foreground_frame,
                _config.block_size,
                _config.use_harris_dtor,
                _config.k);
        }
        catch (const cv::Exception& e) {
            std::cerr << "OpenCV exception: " << e.what() << std::endl; // TODO: change with ROS debug
        }

        _find_contours(_foreground_frame, frame, corners, centers, corners_in_contours);
        return _naive_pose2d(centers, corners_in_contours);
    }

    void BlobDetector::_find_contours(cv::Mat& frame,
        cv::Mat& annot_frame,
        const std::vector<cv::Point2f>& corners,
        std::vector<cv::Point2f>& centers,
        std::vector<std::vector<cv::Point2f>>& corners_in_contours) const
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        try {
            cv::findContours(frame, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        }
        catch (const cv::Exception& e) {
            std::cerr << "OpenCV exception: " << e.what() << std::endl; // TODO: change with ROS debug
        }

        if (_annotate_frame && hierarchy.size() > 0) {
            int idx = 0;
            for (; idx >= 0; idx = hierarchy[idx][0]) {
                cv::drawContours(frame, contours, idx, cv::Scalar(0, 0, 255), cv::FILLED, 8, hierarchy);
            }
        }

        std::vector<std::vector<cv::Point>> contours_poly;
        contours_poly.resize(contours.size());

        std::vector<cv::Point2f> corners_in_contour;
        for (size_t i = 0; i < contours.size(); ++i) {
            cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
            corners_in_contour.clear();
            for (auto& corner : corners) {
                if (cv::pointPolygonTest(contours[i], corner, false) >= 0) {
                    corners_in_contour.push_back(corner);
                }
            }
            if (corners_in_contour.size() > 0) {
                centers.push_back(_contour_center(contours[i]));
                corners_in_contours.push_back(corners_in_contour);
            }
        }

        if (_annotate_frame) {
            for (unsigned int i = 0; i < corners_in_contours.size(); ++i) {
                cv::circle(annot_frame, corners_in_contours[i][0], 3, cv::Scalar(0, 0, 255), -1, 8, 0);
                cv::circle(annot_frame, centers[i], 3, cv::Scalar(255, 0, 0), -1, 8, 0);
            }
        }
    }

    cv::Point2f BlobDetector::_contour_center(const std::vector<cv::Point> contour) const
    {
        cv::Moments m = cv::moments(contour, true);
        return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
    }

    std::vector<cv::Point3f> BlobDetector::_naive_pose2d(const std::vector<cv::Point2f>& centers,
        const std::vector<std::vector<cv::Point2f>>& corners_in_contours) const
    {
        std::vector<cv::Point3f> poses(centers.size());
        for (size_t i = 0; i < poses.size(); ++i) {
            double theta = std::atan2(corners_in_contours[i][0].y - centers[i].y, corners_in_contours[i][0].x - centers[i].x);
            poses[i] = cv::Point3f(centers[i].x, centers[i].y, theta);
        }
        return poses;
    }

    void BlobDetector::reset_background_detector()
    {
        _background_counter = 0;
    }

} // namespace bobi
