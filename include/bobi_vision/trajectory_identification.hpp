#ifndef TRAJECTORY_IDENTIFICATION_HPP
#define TRAJECTORY_IDENTIFICATION_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_vision/filtering_method_base.hpp>

namespace bobi {

    class TrajectoryIdentification {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        TrajectoryIdentification(std::shared_ptr<ros::NodeHandle> nh, size_t matrix_len = 15)
            : _nh(nh),
              _matrix_len(matrix_len)
        {
            // Pose subscribers
            _naive_poses_sub = _nh->subscribe("naive_poses", 10, &TrajectoryIdentification::naive_pose_cb, this);
            _robot_poses_sub = _nh->subscribe("robot_poses", 10, &TrajectoryIdentification::robot_pose_cb, this);
        }

        std::vector<bobi_msgs::PoseStamped> filter()
        {
            _filter->operator()(_filtered_pose_matrix, _filtered_robot_pose_matrix);
            return _filtered_pose_matrix.front();
        }

    protected:
        void naive_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            if (_filtered_pose_matrix.size() > _matrix_len) {
                _filtered_pose_matrix.pop_back();
            }
            _filtered_pose_matrix.push_front(pose_vec_ptr->poses);
        }

        void robot_pose_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec_ptr)
        {
            if (_filtered_robot_pose_matrix.size() > _matrix_len) {
                _filtered_robot_pose_matrix.pop_back();
            }
            _filtered_robot_pose_matrix.push_front(pose_vec_ptr->poses);
        }

        std::shared_ptr<FilteringMethodBase> _filter;
        size_t _matrix_len;
        AgentPoseList _filtered_pose_matrix;
        AgentPoseList _filtered_robot_pose_matrix;

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _naive_poses_sub;
        ros::Subscriber _robot_poses_sub;
    };

} // namespace bobi

#endif
