#include <iostream>
#include <bobi_vision/blob_detector.h>

using namespace bobi;

struct BlobDetectorConfig : public defaults::BlobDetectorConfig {
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

int main(int argc, char** argv)
{
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open cap" << std::endl;
        return 1;
    }

    cv::namedWindow("Fish Detector", cv::WINDOW_AUTOSIZE);

    BlobDetectorConfig cfg;
    BlobDetector bd(cfg, true);
    cv::Mat frame;
    while (true) {
        cap >> frame;

        cv::resize(frame, frame, cv::Size(512, 512));
        std::vector<cv::Point3f> poses2d = bd.detect(frame);

        cv::imshow("Fish Detector", frame);
        if ((cv::waitKey(1) % 256) == 27) {
            break;
        }
    }

    cap.release();
    return 0;
}