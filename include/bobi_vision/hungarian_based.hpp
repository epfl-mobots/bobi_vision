#ifndef BOBI_HUNGARIAN_BASED_HPP
#define BOBI_HUNGARIAN_BASED_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <bobi_vision/hungarian_algorithm.hpp>
#include <bobi_vision/nearest_centroid.hpp>

namespace bobi {

    class HungarianBased : public NearestCentroid {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        HungarianBased(bool force_robot_position = true) : NearestCentroid(force_robot_position)
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

            if (t0->size()) {
                (*t0).clear();
                size_t sz = std::min(num_agents, copy.size());
                for (size_t i = 0; i < sz; ++i) {
                    t0->push_back(copy[i]);
                }
            }

            if (individual_poses.size() > 1) {
                auto t1 = std::next(individual_poses.begin());

                std::vector<std::vector<double>> cost_mat;
                cost_mat.resize(copy.size());
                for (size_t i = 0; i < copy.size(); ++i) {
                    cost_mat[i].resize(t1->size());
                    for (size_t j = 0; j < t1->size(); ++j) {
                        double dist = euc_distance(copy[i], (*t1)[j]);
                        double asim = angle_sim(copy[i], (*t1)[j]);
                        double cost = 0.9 * dist + 0.1 * asim;
                        cost_mat[i][j] = cost;
                    }
                }

                std::vector<int> assignment_idcs;
                _ha.solve(cost_mat, assignment_idcs);

                std::vector<size_t> not_matched;
                t0->clear();
                std::copy(copy.begin(), copy.end(), std::back_inserter(*t0));
                for (size_t i = 0; i < cost_mat.size(); ++i) {
                    if (assignment_idcs[i] >= 0) {
                        (*t0)[assignment_idcs[i]] = copy[i];
                    }
                    else {
                        not_matched.push_back(i);
                    }
                }
            }
        }

    protected:
        algo::HungarianAlgorithm _ha;
    };
} // namespace bobi

#endif
