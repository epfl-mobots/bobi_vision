#ifndef BOBI_NEAREST_CENTROID_HPP
#define BOBI_NEAREST_CENTROID_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <numeric>
#include <algorithm>

namespace bobi {

    class NearestCentroid : public FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        NearestCentroid(bool force_robot_position = true) : FilteringMethodBase(force_robot_position)
        {
            _min_history_len = 2;
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            if (_missing_count.size() != num_agents || _toggle_heading.size() != num_agents) {
                _missing_count = std::vector<int>(num_agents, 0);
                _toggle_heading = std::vector<int>(num_agents, 0);
            }

            if (_missing_robot_count.size() != num_robots) {
                _missing_robot_count = std::vector<int>(num_robots, 0);
            }

            // ------ Handling bottom camera ----------

            int start_idx = 0;
            if (_force_robot_position) {
                int start_idx = num_robots;
            }

            // if the colour detector does not see one or many robots try to fix it
            auto r0 = robot_poses.begin();
            if ((*r0).size() < num_robots) {
                _fill_missing_agents(robot_poses, robot_poses, num_robots); // TODO: in the future we can assume that robot poses are always sorted (using different led colours)

                for (size_t i = 0; i < num_robots; ++i) {
                    bobi_msgs::PoseStamped p = (*r0)[i];
                    if (p.pose.xyz.x != INVALID) {
                        continue;
                    }

                    if (robot_poses.size() == 2) {
                        auto r1 = std::next(robot_poses.begin());
                        bobi_msgs::PoseStamped p1 = (*r1)[i];
                        (*r0)[i] = p1;
                        (*r0)[i].header.stamp = ros::Time::now();
                        (*r0)[i].pose.is_filtered = true;
                    }
                    else if (robot_poses.size() > 2) {
                        auto r1 = std::next(robot_poses.begin());
                        auto r2 = std::next(r1);
                        bobi_msgs::PoseStamped p;

                        p.pose.xyz.x = ((*r1)[i].pose.xyz.x - (*r2)[i].pose.xyz.x) * 1.05 + (*r1)[i].pose.xyz.x;
                        p.pose.xyz.y = ((*r1)[i].pose.xyz.y - (*r2)[i].pose.xyz.y) * 1.05 + (*r1)[i].pose.xyz.y;
                        p.pose.rpy.yaw = _angle_to_pipi(_angle_to_pipi((*r1)[i].pose.rpy.yaw - (*r2)[i].pose.rpy.yaw) * 1.05 + (*r1)[i].pose.rpy.yaw);
                        p.header.stamp = ros::Time::now();
                        p.pose.is_filtered = true;

                        (*r0)[i] = p;
                    }
                }
            }

            // check for nearest centroids and match with past trajectories
            if (num_robots) { // robot pass
                bobi_msgs::PoseStamped inv;
                inv.pose.xyz.x = INVALID;
                inv.pose.xyz.y = INVALID;
                inv.pose.rpy.yaw = INVALID;

                std::vector<int> missing_robot_idcs;

                AgentPose copy(num_agents, inv);
                auto t0 = individual_poses.begin();
                if (t0->size()) {
                    std::copy(t0->begin(), t0->begin() + num_agents, copy.begin());
                }

                for (size_t i = 0; i < r0->size(); ++i) {
                    float min_dist = std::numeric_limits<float>::infinity();
                    int min_idx = INVALID;

                    for (size_t j = 0; j < t0->size(); ++j) {
                        float tolerance = MAX_DIST_PER_TS;
                        float dist = euc_distance((*r0)[i], (*t0)[j]);
                        float angle = angle_sim((*r0)[i], (*t0)[j]);
                        if ((dist < min_dist) && (dist <= tolerance)) {
                            min_dist = dist;
                            min_idx = j;
                        }
                    }

                    if (min_idx != INVALID) {
                        if (i >= t0->size()) {
                            copy.insert(copy.begin() + i, (*r0)[i]);
                            copy[i].pose.is_filtered = true;
                        }
                        else {
                            // make sure robots are always at the beginning of the list ! this is important
                            double yaw = std::abs((*r0)[i].pose.rpy.yaw - (*t0)[min_idx].pose.rpy.yaw);
                            if (yaw > M_PI) {
                                copy[i] = (*r0)[i];
                                copy[i].pose.is_filtered = true;
                                // copy.insert(copy.begin() + i, (*r0)[i]);
                            }
                            else {
                                auto tmp = (*t0)[i];
                                copy[i] = (*t0)[min_idx];
                                copy[min_idx] = tmp;
                                copy[min_idx].pose.is_swapped = true;
                                copy[i].pose.is_swapped = true;

                                // copy[i].pose.rpy.yaw = (*r0)[i].pose.rpy.yaw;
                            }
                        }

                        _missing_robot_count[i] = 0;
                    }
                    else {
                        missing_robot_idcs.push_back(i);
                    }

                    if (missing_robot_idcs.size()) {
                        ROS_WARN("Inconsistent tracking of %ld robots", missing_robot_idcs.size());
                        for (int idx : missing_robot_idcs) {
                            if (_missing_robot_count[idx] < individual_poses.size() - 2 || i < t0->size()) {
                                copy[idx] = (*r0)[idx];
                                copy[idx].pose.is_filtered = true;
                            }
                            else {
                                auto t = std::next(individual_poses.begin(), _missing_robot_count[idx]);
                                copy[idx] = (*t)[idx]; // revert to last valid position to (hopefully) force the behavioural model to recover the lure on its own !! this is not guaranteed
                                copy[idx].header.stamp = ros::Time::now();
                                copy[idx].pose.is_filtered = true;
                            }
                        }
                    }
                }

                t0->resize(copy.size());
                std::copy(copy.begin(), copy.end(), t0->begin());
            }

            // ------ Handling top camera ----------
            AgentPoseList copy(individual_poses.size());
            std::copy(individual_poses.begin(), individual_poses.end(), copy.begin());

            _fill_missing_agents(copy, individual_poses, num_agents);

            // check for nearest centroids and match with past trajectories
            // first pass of assigning ids. Here, missing ids are left for post-inference
            _rearrange_to_nearest_centroid(copy, individual_poses, num_agents, start_idx, num_agents);

            // second pass of assigning ids. Attempt to see if t-1 is close to any other fish from t (perhaps the 2 are too close)
            // _check_overlapping(individual_poses, num_agents, start_idx, num_agents);

            // find missing fish and attempt to estimate/handle the missing position

            _infer_missing(copy, individual_poses, robot_poses, num_agents, num_robots, start_idx, num_agents);

            auto ct0 = copy.begin();
            for (size_t i = 0; i < (*ct0).size(); ++i) {
                bobi_msgs::PoseStamped pi = (*ct0)[i];
                if (pi.pose.xyz.x == INVALID) {
                    ROS_ERROR("invalid got loose");
                }
            }

            auto t0 = individual_poses.begin();
            (*t0).resize(ct0->size());
            std::copy((*ct0).begin(), (*ct0).end(), (*t0).begin());
        }

    protected:
        void _fill_missing_agents(AgentPoseList& copy, const AgentPoseList& l, int n)
        {
            auto t0 = l.begin();
            auto ct0 = copy.begin();

            int missing_ind = n - (int)t0->size();
            if (missing_ind <= 0) {
                return;
            }

            bobi_msgs::PoseStamped inv;
            inv.pose.xyz.x = INVALID;
            inv.pose.xyz.y = INVALID;
            inv.pose.rpy.yaw = INVALID;

            for (size_t i = 0; i < missing_ind; ++i) {
                (*ct0).push_back(inv);
            }
        }

        void _rearrange_to_nearest_centroid(AgentPoseList& copy, const AgentPoseList& l, int num_agents, int sidx, int eidx)
        {
            auto t0 = l.begin();
            auto ct0 = copy.begin();
            auto t1 = std::next(l.begin());

            bobi_msgs::PoseStamped inv;
            inv.pose.xyz.x = INVALID;
            inv.pose.xyz.y = INVALID;
            inv.pose.rpy.yaw = INVALID;

            std::vector<int> taken_idcs;
            _angle_sims.resize(num_agents, std::numeric_limits<float>::infinity());
            _original_idcs.resize(num_agents, INVALID);

            for (size_t i = sidx; i < (*ct0).size(); ++i) {
                float min_dist = std::numeric_limits<float>::infinity();
                int min_idx = INVALID;

                for (size_t j = sidx; j < (*t1).size(); ++j) {
                    float tolerance = MAX_DIST_PER_TS * (_missing_count[j] + 1);

                    float dist = euc_distance((*ct0)[i], (*t1)[j]);
                    float angle = angle_sim((*ct0)[i], (*t1)[j]);
                    if ((dist < min_dist) && (dist <= tolerance)) {
                        min_dist = dist;
                        min_idx = j;
                        _angle_sims[i] = angle;
                    }
                }

                if (min_idx != INVALID) {
                    bool taken = false;
                    for (int idx : taken_idcs) {
                        if (idx == min_idx) {
                            taken = true;
                            break;
                        }
                    }

                    if (!taken) {
                        (*ct0)[min_idx] = (*ct0)[i];
                        if (i != min_idx) {
                            (*ct0)[min_idx].pose.is_swapped = true;
                        }
                        _original_idcs[i] = min_idx;
                        _missing_count[min_idx] = 0;
                        taken_idcs.push_back(min_idx);
                    }
                }
            }

            for (size_t i = 0; i < _original_idcs.size(); ++i) {
                bobi_msgs::PoseStamped pi = (*ct0)[i];
                for (size_t j = 0; j < _original_idcs.size(); ++j) {
                    bobi_msgs::PoseStamped pj = (*ct0)[j];

                    if (i != j && pi.pose.xyz.x == pj.pose.xyz.x && pi.pose.xyz.y == pj.pose.xyz.y && pi.pose.rpy.yaw == pj.pose.rpy.yaw) {
                        int midx = i;
                        if (_angle_sims[j] < _angle_sims[i]) {
                            midx = j;
                        }

                        (*ct0)[midx] = (*ct0)[_original_idcs[midx]];
                        if (i != midx) {
                            (*ct0)[midx].pose.is_swapped = true;
                        }
                        // if (midx < t0->size()) {
                        //     (*ct0)[midx] = (*t0)[midx];
                        // }
                        ++_missing_count[midx];
                    }
                }
            }

            for (size_t i = sidx; i < (*ct0).size(); ++i) {
                bobi_msgs::PoseStamped pi = (*ct0)[i];
                if (pi.pose.xyz.x == INVALID) {
                    if (i < (*t0).size()) {
                        (*ct0)[i] = (*t0)[i];
                    }
                }
            }
        }

        void _check_overlapping(AgentPoseList& l, int num_agents, int sidx, int eidx)
        {
            auto t0 = l.begin();
            auto t1 = std::next(l.begin());

            AgentPose copy(*t0);
            for (size_t i = sidx; i < eidx; ++i) {
                if ((*t0)[i].pose.xyz.x == INVALID && _missing_count[i] == 1) {
                    bobi_msgs::PoseStamped last_valid = (*t1)[i];
                    float min_dist = std::numeric_limits<float>::infinity();
                    int min_idx = INVALID;

                    for (size_t j = 0; j < eidx; ++j) {
                        if ((*t0)[j].pose.xyz.x == INVALID) {
                            continue;
                        }
                        else {
                            float tolerance = MAX_DIST_PER_TS;
                            float dist = euc_distance((*t0)[j], last_valid);
                            float angle = angle_sim((*t0)[j], last_valid);
                            if ((dist < min_dist) && (dist <= tolerance)) {
                                min_dist = dist;
                                min_idx = j;
                            }
                        }
                    }
                    if (min_idx != INVALID) {
                        copy[i] = (*t0)[min_idx];
                        if (i != min_idx) {
                            copy[i].pose.is_swapped = true;
                        }
                    }
                }
            }

            t0->resize(num_agents);
            std::copy(copy.begin(), copy.end(), t0->begin());
        }

        void _infer_missing(AgentPoseList& copy, const AgentPoseList& il, const AgentPoseList& rl, int num_agents, int num_robots, int sidx, int eidx)
        {
            auto t0 = il.begin();
            auto ct0 = copy.begin();
            auto t1 = std::next(il.begin());

            for (size_t i = sidx; i < eidx; ++i) {
                if ((*ct0)[i].pose.xyz.x == INVALID) {
                    ++_missing_count[i];
                    if (il.size() > 2 && _missing_count[i] <= 1) {
                        auto t2 = il.begin();
                        std::advance(t2, 2);
                        bobi_msgs::PoseStamped p;
                        p.pose.xyz.x = ((*t1)[i].pose.xyz.x - (*t2)[i].pose.xyz.x) + (*t1)[i].pose.xyz.x;
                        p.pose.xyz.y = ((*t1)[i].pose.xyz.y - (*t2)[i].pose.xyz.y) + (*t1)[i].pose.xyz.y;
                        p.pose.rpy.yaw = _angle_to_pipi(_angle_to_pipi((*t1)[i].pose.rpy.yaw - (*t2)[i].pose.rpy.yaw) + (*t1)[i].pose.rpy.yaw);
                        p.header.stamp = ros::Time::now();
                        p.pose.is_filtered = true;

                        (*ct0)[i] = p;
                    }
                    else {
                        if (i < (*t0).size()) {
                            (*ct0)[i] = (*t0)[i];
                        }
                        else {
                            (*ct0)[i] = (*t1)[i];
                            (*ct0)[i].pose.is_filtered = true;
                        }
                        (*ct0)[i].header.stamp = ros::Time::now();
                    }
                }

                // if angle change is too big then we can assume that the tracking system
                // is reporting the angle + 180. We correct this change by toggling the angle
                // back to what we think is the correct value

                //this check is skipped if i is a robot
                if (i < num_robots) {
                    continue;
                }

                float abs_angle_change = abs(_angle_to_pipi((*ct0)[i].pose.rpy.yaw - (*t1)[i].pose.rpy.yaw));
                if (abs_angle_change > 5. * M_PI / 6.) {
                    if (_toggle_heading[i] < 5) {
                        // (*t0)[i].pose.rpy.yaw = -(*t0)[i].pose.rpy.yaw;
                        (*ct0)[i].pose.rpy.yaw = (*t1)[i].pose.rpy.yaw;
                        ++_toggle_heading[i];
                    }
                    else {
                        _toggle_heading[i] = 0;
                    }
                }
                else {
                    _toggle_heading[i] = 0;
                }
            }
        }

        std::vector<int> _missing_count;
        std::vector<int> _missing_robot_count;
        std::vector<int> _toggle_heading;

        std::vector<float> _angle_sims;
        std::vector<float> _original_idcs;
    };
} // namespace bobi

#endif
