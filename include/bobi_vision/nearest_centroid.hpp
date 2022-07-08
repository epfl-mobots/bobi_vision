#ifndef BOBI_NEAREST_CENTROID_HPP
#define BOBI_NEAREST_CENTROID_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <numeric>
#include <algorithm>

#define INVALID -1000
#define MAX_BL_PER_TS 0.015

namespace bobi {

    class NearestCentroid : public FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {

            if (_missing_count.size() != num_agents || _toggle_heading.size() != num_agents) {
                _missing_count = std::vector<int>(num_agents, 0);
                _toggle_heading = std::vector<bool>(num_agents, false);
            }

            auto euc_distance = [](const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o) {
                return std::sqrt(
                    std::pow(o.pose.xyz.x - n.pose.xyz.x, 2.)
                    + std::pow(o.pose.xyz.y - n.pose.xyz.y, 2.));
            };

            auto angle_sim = [&](const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o) {
                return std::abs(_angle_to_pipi(n.pose.rpy.yaw - o.pose.rpy.yaw));
            };

            auto t0 = individual_poses.begin();
            auto t1 = individual_poses.begin();
            std::advance(t1, 1);
            size_t missing_ind = std::abs((int)t0->size() - (int)t1->size());
            for (size_t i = 0; i < missing_ind; ++i) {
                bobi_msgs::PoseStamped p;
                p.pose.xyz.x = INVALID;
                p.pose.xyz.y = INVALID;
                p.pose.rpy.yaw = INVALID;
                (*t0).push_back(p);
            }

            // check for nearest centroids and match with past trajectories
            { // first pass of assigning ids. Here, missing ids are left for post-inference
                bobi_msgs::PoseStamped inv;
                inv.pose.xyz.x = INVALID;
                inv.pose.xyz.y = INVALID;
                inv.pose.rpy.yaw = INVALID;
                AgentPose copy(num_agents, inv);
                for (size_t i = 0; i < t0->size(); ++i) {

                    float min_dist = std::numeric_limits<float>::infinity();
                    int min_idx = INVALID;

                    for (size_t j = 0; j < t1->size(); ++j) {
                        float tolerance = MAX_BL_PER_TS * (_missing_count[j] + 1);
                        float dist = euc_distance((*t0)[i], (*t1)[j]);
                        float angle = angle_sim((*t0)[i], (*t1)[j]);
                        if ((dist < min_dist) && (dist <= tolerance)) {
                            min_dist = dist;
                            min_idx = j;
                        }
                    }
                    if (min_idx != INVALID) {
                        copy[min_idx] = (*t0)[i];
                        _missing_count[min_idx] = 0;
                    }
                }
                t0->resize(num_agents);
                std::copy(copy.begin(), copy.end(), t0->begin());
            }

            { // second pass of assigning ids. Attempt to see if t-1 is close to any other fish from t (perhaps the 2 are too close)
                AgentPose copy(*t0);
                for (size_t i = 0; i < t0->size(); ++i) {
                    if ((*t0)[i].pose.xyz.x == INVALID && _missing_count[i] == 1) {
                        bobi_msgs::PoseStamped last_valid = (*t1)[i];
                        float min_dist = std::numeric_limits<float>::infinity();
                        int min_idx = INVALID;

                        for (size_t j = 0; j < t0->size(); ++j) {
                            if ((*t0)[j].pose.xyz.x == INVALID) {
                                continue;
                            }
                            else {
                                float tolerance = MAX_BL_PER_TS;
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
                        }
                    }
                }

                t0->resize(num_agents);
                std::copy(copy.begin(), copy.end(), t0->begin());
            }

            // find missing fish and attempt to estimate/handle the missing position
            for (size_t i = 0; i < t0->size(); ++i) {
                if ((*t0)[i].pose.xyz.x == INVALID) {
                    ++_missing_count[i];
                    (*t0)[i] = (*t1)[i];
                    if (individual_poses.size() > 2 && _missing_count[i] <= 2) {
                        auto t2 = individual_poses.begin();
                        std::advance(t2, 2);
                        bobi_msgs::PoseStamped p;
                        p.pose.xyz.x = ((*t1)[i].pose.xyz.x - (*t2)[i].pose.xyz.x) + (*t1)[i].pose.xyz.x;
                        p.pose.xyz.y = ((*t1)[i].pose.xyz.y - (*t2)[i].pose.xyz.y) + (*t1)[i].pose.xyz.y;
                        p.pose.rpy.yaw = _angle_to_pipi(_angle_to_pipi((*t1)[i].pose.rpy.yaw - (*t2)[i].pose.rpy.yaw) + (*t1)[i].pose.rpy.yaw);

                        if (euc_distance(p, (*t0)[i]) <= MAX_BL_PER_TS) {
                            (*t0)[i] = p;
                        }
                    }
                }

                // if angle change is too big then we can assume that the tracking system
                // is reporting the angle + 180. We correct this change by toggling the angle
                // back to what we think is the correct value
                float abs_angle_change = abs(_angle_to_pipi((*t0)[i].pose.rpy.yaw - (*t1)[i].pose.rpy.yaw));
                if (abs_angle_change > 5. * M_PI / 6.) {
                    if (_toggle_heading[i]) {
                        _toggle_heading[i] = false;
                    }
                    else {
                        (*t0)[i].pose.rpy.yaw = _angle_to_pipi((*t0)[i].pose.rpy.yaw + M_PI);
                        _toggle_heading[i] = true;
                    }
                }
                else {
                    _toggle_heading[i] = false;
                }
            }
        }

    protected:
        double _angle_to_pipi(double angle)
        {
            while (true) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
                if (abs(angle) <= M_PI) {
                    break;
                }
            }
            return angle;
        }

        std::vector<int> _missing_count;
        std::vector<bool> _toggle_heading;
    };
} // namespace bobi

#endif
