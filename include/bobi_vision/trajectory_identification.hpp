#ifndef BOBI_TRAJECTORY_IDENTIFICATION_HPP
#define BOBI_TRAJECTORY_IDENTIFICATION_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_vision/filtering_method_base.hpp>
#include <dynamic_reconfigure/server.h>

#include <bobi_vision/TrajectoryIdentificationConfig.h>
#include <bobi_vision/BlobDetectorConfig.h>
#include <bobi_vision/trajectory_identification_method_factory.hpp>

#include <mutex>

namespace bobi {

    class TrajectoryIdentification {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        TrajectoryIdentification(std::shared_ptr<ros::NodeHandle> nh, size_t matrix_len = 15)
            : _nh(nh),
              _matrix_len(matrix_len),
              _filter(new FilteringMethodBase())
        {
            dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig>::CallbackType f;
            f = boost::bind(&TrajectoryIdentification::_config_cb, this, _1, _2);
            _config_server.setCallback(f);

            // Pose subscribers
            _naive_poses_sub = _nh->subscribe("naive_poses", 1, &TrajectoryIdentification::_naive_pose_cb, this);
            _robot_poses_sub = _nh->subscribe("robot_poses", 1, &TrajectoryIdentification::_robot_pose_cb, this);
        }

        std::vector<bobi_msgs::PoseStamped> filter()
        {
            std::lock_guard<std::mutex> g1(_cfg_mutex);
            std::lock_guard<std::mutex> g2(_rb_mutex);
            std::lock_guard<std::mutex> g3(_ind_mutex);

            if (_filtered_pose_list.size()) {
                _filter->operator()(_filtered_pose_list, _filtered_robot_pose_list, _num_agents, _num_robots);
                return _filtered_pose_list.front();
            }
            else {
                return std::vector<bobi_msgs::PoseStamped>();
            }
        }

    protected:
        void _naive_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_ind_mutex);
            if ((_filtered_pose_list.size() == 0)
                && (pose_vec_ptr->poses.size() != (_num_agents - _num_robots))) {
                return;
            }
            if (pose_vec_ptr->poses.size() == 0) {
                return;
            }
            if (_filtered_pose_list.size() > _matrix_len) {
                _filtered_pose_list.pop_back();
            }
            _filtered_pose_list.push_front(pose_vec_ptr->poses);
        }

        void _robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_rb_mutex);
            if (_filtered_robot_pose_list.size() > _matrix_len) {
                _filtered_robot_pose_list.pop_back();
            }
            _filtered_robot_pose_list.push_front(pose_vec_ptr->poses);
        }

        void _config_cb(bobi_vision::TrajectoryIdentificationConfig& config, uint32_t level)
        {
            std::lock_guard<std::mutex> guard(_cfg_mutex);
            bool success;
            _num_agents = config.num_agents;
            _num_robots = config.num_robots;
            std::tie(_filter, success) = _method_factory(config.method);
            if (success) {
                ROS_INFO("Trajectory Identification method changed");
                _filtered_pose_list.clear();
                _filtered_robot_pose_list.clear();
            }
            else {
                ROS_INFO("Using default Trajectory Identification method");
            }
        }

        size_t _num_agents;
        size_t _num_robots;

        std::shared_ptr<FilteringMethodBase> _filter;
        size_t _matrix_len;
        AgentPoseList _filtered_pose_list;
        AgentPoseList _filtered_robot_pose_list;

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _naive_poses_sub;
        ros::Subscriber _robot_poses_sub;
        std::mutex _ind_mutex;
        std::mutex _rb_mutex;

        dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig> _config_server;
        std::mutex _cfg_mutex;
        TrajectoryIdentificationMethodFactory _method_factory;
    };

} // namespace bobi

#endif
