#ifndef BOBI_NEAREST_CENTROID_HPP
#define BOBI_NEAREST_CENTROID_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

namespace bobi {

    class NearestCentroid : public FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        NearestCentroid(bool force_robot_position = true) : FilteringMethodBase(force_robot_position)
        {
            _min_history_len = 1;
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            auto r0 = robot_poses.begin();
            auto t0 = individual_poses.begin();

            AgentPose copy;
            if (t0->size()) {
                copy.resize(t0->size());
                std::copy(t0->begin(), t0->end(), copy.begin());
            }
            else {
                if (num_robots && r0->size()) {
                    copy.resize(r0->size());
                    t0->resize(r0->size());
                    std::copy(r0->begin(), r0->end(), copy.begin());
                    std::copy(r0->begin(), r0->end(), t0->begin());
                }
            }

            size_t sidx = num_robots;
            if (num_robots > 0 && r0->size()) {
                if (copy.size() >= num_agents) {
                    _rearrange_to_nearest_centroids(copy, *r0, *t0, 0, true);
                }
                else {
                    auto distances = _rearrange_to_nearest_centroids(copy, *r0, *t0, 0, true);

                    if (num_robots == 1 && (distances[0] > MAX_DIST_PER_TS || distances[0] == INVALID)
                        && individual_poses.size() > 1) {

                        auto t1 = std::next(individual_poses.begin());
                        if (t1->size()) {
                            _rearrange_to_nearest_centroids(copy, *r0, *t1, 0, true);
                        }
                    }
                }
            }

            // if (individual_poses.size() > 1) {
            //     auto t1 = std::next(individual_poses.begin());
            //     _rearrange_to_nearest_centroids(copy, *t0, *t1, num_robots, false);
            // }

            for (size_t i = 0; i < copy.size(); ++i) {
                if (copy[i].pose.xyz.x == INVALID) {
                    ROS_ERROR("Invalid in poses");
                }
            }

            if (t0->size()) {
                (*t0).clear();
                size_t sz = std::min(num_agents, copy.size());
                for (size_t i = 0; i < sz; ++i) {
                    t0->push_back(copy[i]);
                }
            }
        }

    protected:
        std::vector<float> _rearrange_to_nearest_centroids(AgentPose& copy, const AgentPose& l1, const AgentPose& l2, int sidx, bool is_robot = true)
        {
            float min_dist = std::numeric_limits<float>::infinity();
            int min_idx = INVALID;

            std::vector<bool> taken_idcs(l2.size(), false);
            std::vector<float> distances(l2.size(), INVALID);

            for (size_t i = sidx; i < l1.size(); ++i) {
                for (size_t j = sidx; j < l2.size(); ++j) {
                    float dist = euc_distance(l1[i], l2[j]);
                    if (dist < min_dist
                        && !taken_idcs[j]) {
                        if (min_idx < l2.size()) {
                            taken_idcs[min_idx] = false;
                        }
                        taken_idcs[j] = true;
                        min_dist = dist;
                        min_idx = j;
                    }
                }

                if (min_idx != INVALID) {
                    double yaw = std::abs(l1[i].pose.rpy.yaw - l2[min_idx].pose.rpy.yaw);

                    copy[min_idx] = l2[i];
                    copy[i] = l2[min_idx];
                    copy[i].pose.is_swapped = true;
                    if (yaw > M_PI && is_robot) {
                        copy[i].pose.rpy.yaw = l1[i].pose.rpy.yaw;
                        copy[i].pose.is_filtered = true;
                    }

                    distances[min_idx] = min_dist;
                }
            }

            return distances;
        }
    };
} // namespace bobi

#endif
