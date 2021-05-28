#include <ros/ros.h>

#include <bobi_vision/frame_info.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

#include <iostream>

using namespace bobi;

class RawImageWrapper {
public:
    RawImageWrapper(std::shared_ptr<ros::NodeHandle> nh)
        : _nh(nh),
          _it(*_nh)
    {
        _frame_sub = _it.subscribe("top_camera/image_undistorted", 1, &RawImageWrapper::und_image_cb, this);
        _bottom_frame_sub = _it.subscribe("bottom_camera/image_undistorted", 1, &RawImageWrapper::bottom_und_image_cb, this);
        _poses_sub = _nh->subscribe("filtered_poses", 10, &RawImageWrapper::filtered_pose_cb, this);
        _robot_poses_sub = _nh->subscribe("robot_poses", 10, &RawImageWrapper::robot_pose_cb, this);

        _nh->param<double>("top_camera/pix2m", _top_pix2m, 0.001475);
        _nh->param<double>("bottom_camera/pix2m", _bottom_pix2m, 0.002681818182);
        _fi.set_top_pix2m(_top_pix2m);
        _fi.set_bottom_pix2m(_bottom_pix2m);
    }

protected:
    void und_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

            _fi.draw_all(frame, _individual_poses, _robot_poses);

            cv::imshow("Individual Tracking", frame);
            cv::waitKey(10);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
        }
    }

    void bottom_und_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            _fi.draw_all(frame, _individual_poses, _robot_poses);

            cv::imshow("Robot Tracking", frame);
            cv::waitKey(10);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void filtered_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
    {
        _individual_poses = pose_vec_ptr->poses;
    }

    void robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
    {
        _robot_poses = pose_vec_ptr->poses;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    image_transport::ImageTransport _it;

    std::vector<bobi_msgs::PoseStamped> _individual_poses;
    std::vector<bobi_msgs::PoseStamped> _robot_poses;

    image_transport::Subscriber _frame_sub;
    image_transport::Subscriber _bottom_frame_sub;
    ros::Subscriber _poses_sub;
    ros::Subscriber _robot_poses_sub;

    double _top_pix2m;
    double _bottom_pix2m;

    FrameInfo _fi;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    RawImageWrapper riw(nh);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow("BOBI Framework");

    return 0;
}