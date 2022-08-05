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
        TrajectoryIdentification(std::shared_ptr<ros::NodeHandle> nh, size_t matrix_len = 5)
            : _nh(nh),
              _matrix_len(matrix_len),
              _filter(new FilteringMethodBase()),
              _init_successful(false)
        {
            dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig>::CallbackType f;
            f = boost::bind(&TrajectoryIdentification::_config_cb, this, _1, _2);
            _config_server.setCallback(f);

            // Pose subscribers
            _naive_poses_sub = _nh->subscribe("naive_poses", 1, &TrajectoryIdentification::_naive_pose_cb, this);
            _robot_poses_sub = _nh->subscribe("robot_poses", 1, &TrajectoryIdentification::_robot_pose_cb, this);
            _bottom2top_srv = _nh->serviceClient<bobi_msgs::ConvertCoordinates>("convert_bottom2top");
        }

        std::vector<bobi_msgs::PoseStamped> filter()
        {
            std::lock_guard<std::mutex> g1(_cfg_mutex);
            std::lock_guard<std::mutex> g2(_rb_mutex);
            std::lock_guard<std::mutex> g3(_ind_mutex);

            if (_filtered_pose_list.size() > 1) {
                _filter->operator()(_filtered_pose_list, _filtered_robot_pose_list, _num_agents, _num_robots);
            }

            if (_filtered_pose_list.size()) {
                return _filtered_pose_list.front();
            }
            else {
                return std::vector<bobi_msgs::PoseStamped>();
            }
        }

        AgentPoseList filtered_list()
        {
            return _filtered_pose_list;
        }

    protected:
        void _naive_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_ind_mutex);

            std::vector<bobi_msgs::PoseStamped> copies(pose_vec_ptr->poses.size());
            std::copy(pose_vec_ptr->poses.begin(), pose_vec_ptr->poses.end(), copies.begin());

            if (!_init_successful) {
                if (copies.size() == _num_agents) {
                    _filtered_pose_list.push_front(copies);
                }
                else {
                    _filtered_pose_list.clear();
                }

                if (_filtered_pose_list.size() == 2) {
                    _init_successful = true;
                    // _filter->operator()(_filtered_pose_list, _filtered_robot_pose_list, _num_agents, _num_robots);
                }
            }
            else {
                _filtered_pose_list.push_front(copies);
                if (_filtered_pose_list.size() > _matrix_len) {
                    _filtered_pose_list.pop_back();
                }
                // _filter->operator()(_filtered_pose_list, _filtered_robot_pose_list, _num_agents, _num_robots);
            }

            if (_init_successful && (_filtered_pose_list.size() < 2)) {
                _init_successful = false;
                _filtered_pose_list.clear();
            }
        }

        void _robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            std::lock_guard<std::mutex> guard(_rb_mutex);

            std::vector<bobi_msgs::PoseStamped> copies(pose_vec_ptr->poses.size());
            std::copy(pose_vec_ptr->poses.begin(), pose_vec_ptr->poses.end(), copies.begin());
            for (size_t i = 0; i < copies.size(); ++i) {
                geometry_msgs::Point converted_p;
                copies[i].pose.xyz = _convert_bottom2top(copies[i].pose.xyz);
            }

            _filtered_robot_pose_list.push_front(copies);

            if (_filtered_robot_pose_list.size() > _matrix_len) {
                _filtered_robot_pose_list.pop_back();
            }
        }

        void _config_cb(bobi_vision::TrajectoryIdentificationConfig& config, uint32_t level)
        {
            std::lock_guard<std::mutex> guard(_cfg_mutex);
            bool success;
            _force_robot_position = config.force_robot_position;
            _num_agents = config.num_agents;
            _num_robots = config.num_robots;
            _num_virtu_agents = config.num_virtu_agents;
            std::tie(_filter, success) = _method_factory(_nh, config.method, _force_robot_position);
            if (success) {
                ROS_INFO("Trajectory Identification method changed");
                _filtered_pose_list.clear();
                _filtered_robot_pose_list.clear();
            }
            else {
                ROS_INFO("Using default Trajectory Identification method");
            }
        }

        geometry_msgs::Point _convert_bottom2top(const geometry_msgs::Point& p)
        {
            bobi_msgs::ConvertCoordinates conv;
            conv.request.p = p;
            if (!_bottom2top_srv.call(conv)) {
                conv.response.converted_p.x = -1;
                conv.response.converted_p.y = -1;
            }
            return conv.response.converted_p;
        }

        size_t _num_agents;
        size_t _num_robots;
        size_t _num_virtu_agents;
        bool _force_robot_position;
        bool _init_successful;

        std::shared_ptr<FilteringMethodBase> _filter;
        size_t _matrix_len;
        AgentPoseList _filtered_pose_list;
        AgentPoseList _filtered_robot_pose_list;

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _naive_poses_sub;
        ros::Subscriber _robot_poses_sub;
        ros::ServiceClient _bottom2top_srv;
        std::mutex _ind_mutex;
        std::mutex _rb_mutex;

        dynamic_reconfigure::Server<bobi_vision::TrajectoryIdentificationConfig> _config_server;
        std::mutex _cfg_mutex;
        TrajectoryIdentificationMethodFactory _method_factory;
    };

} // namespace bobi

#endif
