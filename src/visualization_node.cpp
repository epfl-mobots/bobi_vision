#include <ros/ros.h>

#include <bobi_vision/frame_info.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_vision/mask_factory.hpp>

#include <iostream>
#include <map>

using namespace bobi;

class RawImageWrapper {
public:
    RawImageWrapper(std::shared_ptr<ros::NodeHandle> nh)
        : _nh(nh),
          _it(*_nh),
          _fi(_nh)
    {
        _frame_sub = _it.subscribe("top_camera/image_undistorted", 1, &RawImageWrapper::_top_und_image_cb, this);
        _bottom_frame_sub = _it.subscribe("bottom_camera/image_undistorted", 1, &RawImageWrapper::bottom_und_image_cb, this);
        _poses_sub = _nh->subscribe("filtered_poses", 1, &RawImageWrapper::_filtered_pose_cb, this);
        _robot_poses_sub = _nh->subscribe("robot_poses", 1, &RawImageWrapper::_robot_pose_cb, this);
        _target_pos_sub = _nh->subscribe("target_position", 1, &RawImageWrapper::_target_pos_cb, this);
        _robot_annot_image_pub = _it.advertise("bottom_camera/image_annot", 1);
        _individual_annot_image_pub = _it.advertise("top_camera/image_annot", 1);

        _nh->param<double>("top_camera/pix2m", _top_pix2m, 0.001475);
        _nh->param<double>("bottom_camera/pix2m", _bottom_pix2m, 0.002681818182);
        _fi.set_top_pix2m(_top_pix2m);
        _fi.set_bottom_pix2m(_bottom_pix2m);

        // top camera info
        int top_camera_px_width;
        int top_camera_px_height;
        nh->param<int>("top_camera/camera_px_width_undistorted", top_camera_px_width, 512);
        nh->param<int>("top_camera/camera_px_height_undistorted", top_camera_px_height, 512);

        std::string top_mask_type;
        std::vector<int> top_mask_specs;
        nh->param<std::string>("top_camera/mask_type", top_mask_type, top_mask_type);
        nh->param<std::vector<int>>("top_camera/mask", top_mask_specs, top_mask_specs);
        _fi.set_top_mask(
            bobi::MaskFactory()(
                top_mask_type, top_mask_specs, cv::Size(top_camera_px_width, top_camera_px_height), CV_8UC3));

        // bottom camera info
        int bottom_camera_px_width;
        int bottom_camera_px_height;
        nh->param<int>("bottom_camera/camera_px_width_undistorted", bottom_camera_px_width, 640);
        nh->param<int>("bottom_camera/camera_px_height_undistorted", bottom_camera_px_height, 480);

        std::string bottom_mask_type;
        std::vector<int> bottom_mask_specs;
        nh->param<std::string>("bottom_camera/mask_type", bottom_mask_type, bottom_mask_type);
        nh->param<std::vector<int>>("bottom_camera/mask", bottom_mask_specs, bottom_mask_specs);
        _fi.set_bottom_mask(
            bobi::MaskFactory()(
                bottom_mask_type, bottom_mask_specs, cv::Size(bottom_camera_px_width, bottom_camera_px_height), CV_8UC3));

        // init target vars
        _target_position.pose.xyz.x = -1;
        _target_position.pose.xyz.y = -1;
    }

protected:
    void _top_und_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

            _fi.draw_all(frame, _individual_poses, _robot_poses, _target_position, _ipose_filtered, _rpose_filtered, bobi::CameraLocation::TOP);

            sensor_msgs::ImagePtr image_ptr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            _individual_annot_image_pub.publish(image_ptr);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
        }
    }

    void bottom_und_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            _fi.draw_all(frame, _individual_poses, _robot_poses, _target_position, _ipose_filtered, _rpose_filtered, bobi::CameraLocation::BOTTOM);

            sensor_msgs::ImagePtr image_ptr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            _robot_annot_image_pub.publish(image_ptr);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void _filtered_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
    {
        _individual_poses = pose_vec_ptr->poses;
        _ipose_filtered = pose_vec_ptr->is_filtered;
    }

    void _robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
    {
        _robot_poses = pose_vec_ptr->poses;
        _rpose_filtered = pose_vec_ptr->is_filtered;
    }

    void _target_pos_cb(const bobi_msgs::PoseStamped::ConstPtr& pos_ptr)
    {
        _target_position = *pos_ptr;
    }

    std::shared_ptr<ros::NodeHandle> _nh;
    image_transport::ImageTransport _it;

    std::vector<bobi_msgs::PoseStamped> _individual_poses;
    std::vector<bobi_msgs::PoseStamped> _robot_poses;
    bool _ipose_filtered;
    bool _rpose_filtered;

    bobi_msgs::PoseStamped _target_position; // TODO: this assumes a single robot and should be fixed in future versions

    image_transport::Subscriber _frame_sub;
    image_transport::Subscriber _bottom_frame_sub;
    ros::Subscriber _poses_sub;
    ros::Subscriber _robot_poses_sub;
    ros::Subscriber _target_pos_sub;
    image_transport::Publisher _robot_annot_image_pub;
    image_transport::Publisher _individual_annot_image_pub;

    double _top_pix2m;
    double _bottom_pix2m;

    FrameInfo _fi;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    RawImageWrapper riw(nh);
    ros::spin();

    return 0;
}