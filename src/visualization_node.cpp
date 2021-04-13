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
        _frame_sub = _it.subscribe("top_camera/image_raw", 1, &RawImageWrapper::raw_image_cb, this);
        _poses_sub = _nh->subscribe("filtered_poses", 10, &RawImageWrapper::filtered_pose_cb, this);
        _robot_poses_sub = _nh->subscribe("robot_poses", 10, &RawImageWrapper::robot_pose_cb, this);
    }

protected:
    void raw_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

            _fi.draw_all(frame, _individual_poses, _robot_poses);

            cv::imshow("BOBI Framework", frame);
            cv::waitKey(30);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
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
    ros::Subscriber _poses_sub;
    ros::Subscriber _robot_poses_sub;

    FrameInfo _fi;
};

void raw_image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;

        cv::imshow("BOBI Framework", frame);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    cv::namedWindow("BOBI Framework");
    RawImageWrapper riw(nh);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow("BOBI Framework");

    return 0;
}