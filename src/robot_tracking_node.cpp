#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

struct BottomCameraConfig {
    bool using_file = false;
    std::string camera_dev_no = "3";
    int camera_px_width = 640;
    int camera_px_height = 480;
    int fps = 30;
    std::vector<double> camera_matrix = {
        354.63806, 0., 325.08967,
        0., 354.95442, 234.85626,
        0., 0., 1.};
    std::vector<double> distortion_coeffs = {-0.286276, 0.063449, 0.000008, -0.000346, 0.000000};
};

BottomCameraConfig get_camera_config(const ros::NodeHandle& nh)
{
    // configuration for the top camera
    BottomCameraConfig bottom_camera;
    nh.param<bool>("bottom_camera/using_file", bottom_camera.using_file, bottom_camera.using_file);
    nh.param<std::string>("bottom_camera/camera_dev_no", bottom_camera.camera_dev_no, bottom_camera.camera_dev_no);
    nh.param<int>("bottom_camera/camera_px_width", bottom_camera.camera_px_width, bottom_camera.camera_px_width);
    nh.param<int>("bottom_camera/camera_px_height", bottom_camera.camera_px_height, bottom_camera.camera_px_height);
    nh.param<int>("bottom_camera/fps", bottom_camera.fps, bottom_camera.fps);
    nh.param<std::vector<double>>("bottom_camera/camera_matrix", bottom_camera.camera_matrix, bottom_camera.camera_matrix);
    nh.param<std::vector<double>>("bottom_camera/distortion_coefficients", bottom_camera.distortion_coeffs, bottom_camera.distortion_coeffs);
    return bottom_camera;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // get camera config from the parameter server
    BottomCameraConfig camera_cfg = get_camera_config(nh);

    // set opencv stream
    std::string medium;
    cv::VideoCapture camera;

    if (camera_cfg.using_file) {
        camera = cv::VideoCapture(camera_cfg.camera_dev_no);
        medium = "Using video file: " + camera_cfg.camera_dev_no;
    }
    else {
        camera = cv::VideoCapture(std::stoi(camera_cfg.camera_dev_no), cv::CAP_GSTREAMER);
        medium = "Using camera device: " + camera_cfg.camera_dev_no;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_cfg.camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_cfg.camera_px_height);

    ROS_INFO("%s", medium.c_str());
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera/file");
        return 1;
    }

    // publisher for the image_transport wrapped image
    image_transport::Publisher raw_image_pub = it.advertise("bottom_camera/image_raw", 1);
    image_transport::Publisher undistorted_image_pub = it.advertise("bottom_camera/image_undistorted", 1);
    image_transport::Publisher blob_pub = it.advertise("bottom_camera/image_blobs", 1);

    // pose publisher
    ros::Publisher pose_pub;
    pose_pub = nh.advertise<bobi_msgs::PoseVec>("robot_poses", 1);

    cv::Mat distortion_coeffs = cv::Mat(1, 5, CV_64F);
    memcpy(distortion_coeffs.data, camera_cfg.distortion_coeffs.data(), camera_cfg.distortion_coeffs.size() * sizeof(double));
    cv::Mat camera_mat = cv::Mat(3, 3, CV_64F);
    memcpy(camera_mat.data, camera_cfg.camera_matrix.data(), camera_cfg.camera_matrix.size() * sizeof(double));

    cv::Rect roi;
    cv::Mat new_camera_mat = cv::getOptimalNewCameraMatrix(camera_mat, distortion_coeffs, cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), 1., cv::Size(camera_cfg.camera_px_width, camera_cfg.camera_px_height), &roi);

    cv::Mat frame;
    cv::Mat frame_und;
    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        camera >> frame;
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        cv::undistort(frame, frame_und, camera_mat, distortion_coeffs, new_camera_mat);
        frame_und = frame_und(roi);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        // publish raw image
        sensor_msgs::ImagePtr raw_image_ptr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_image_ptr);

        sensor_msgs::ImagePtr undistorted_image_ptr = cv_bridge::CvImage(header, "bgr8", frame_und).toImageMsg();
        undistorted_image_pub.publish(undistorted_image_ptr);

        bobi_msgs::PoseVec pv;
        pose_pub.publish(pv);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}