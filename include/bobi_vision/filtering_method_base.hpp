#ifndef BOBI_FILTERING_METHOD_BASE_HPP
#define BOBI_FILTERING_METHOD_BASE_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/ConvertCoordinates.h>

#include <numeric>

#define INVALID -1000
#define BL 3.5
#define MAX_DIST_PER_TS 0.01
#define MIN_DIST_PER_DUPL 0.003

namespace bobi {
    class FilteringMethodBase {
    public:
        using AgentPose = std::vector<bobi_msgs::PoseStamped>;
        using AgentPoseList = std::list<AgentPose>;

        FilteringMethodBase(bool force_robot_position = false) : _force_robot_position(force_robot_position)
        {
            std::srand(time(NULL));
            _min_history_len = 1;
        }

        float euc_distance(const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o)
        {
            return std::sqrt(
                std::pow(o.pose.xyz.x - n.pose.xyz.x, 2.)
                + std::pow(o.pose.xyz.y - n.pose.xyz.y, 2.));
        }

        float angle_sim(const bobi_msgs::PoseStamped& n, const bobi_msgs::PoseStamped& o)
        {
            return std::abs(_angle_to_pipi(n.pose.rpy.yaw - o.pose.rpy.yaw));
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            AgentPose copy;
            auto r0 = robot_poses.begin();
            auto t0 = individual_poses.begin();
            if (t0->size()) {
                copy.resize(t0->size());
                std::copy(t0->begin(), t0->end(), copy.begin());
            }

            for (size_t i = 0; i < num_robots; ++i) {

                if (_force_robot_position) {
                    copy.insert(copy.begin() + i, (*r0)[i]);
                }

                float min_dist = std::numeric_limits<float>::infinity();
                int min_idx = INVALID;

                for (size_t j = 0; j < t0->size(); ++j) {
                    float tolerance = MAX_DIST_PER_TS;
                    float dist = euc_distance((*r0)[i], (*t0)[j]);
                    float angle = angle_sim((*r0)[i], (*t0)[j]);
                    if ((dist < min_dist) && (dist < tolerance)) {
                        min_dist = dist;
                        min_idx = j;
                    }
                }

                if (min_idx != INVALID) {
                    // make sure robots are always at the beginning of the list ! this is important
                    double yaw = std::abs((*r0)[i].pose.rpy.yaw - copy[min_idx].pose.rpy.yaw);
                    if (yaw > M_PI) {
                        copy[i] = (*r0)[i];
                        // copy.insert(copy.begin() + i, (*r0)[i]);
                    }
                    else {
                        auto tmp = (*t0)[i];
                        copy[i] = (*t0)[min_idx];
                        copy[min_idx] = tmp;
                        copy[i].pose.rpy.yaw = (*r0)[i].pose.rpy.yaw; // the yaw info from the robot tracker is more accurate
                    }
                }
                else {
                    copy.insert(copy.begin() + i, (*r0)[i]);
                }
            }

            _remove_duplicates(copy, num_robots);

            // the following would really not be adequate for more robots
            if (copy.size() > num_agents && num_robots == 1) {
                _keep_more_distant(copy, *r0, num_agents, num_robots);
            }

            t0->resize(copy.size());
            std::copy(copy.begin(), copy.end(), t0->begin());
        }

        int min_history_len() const
        {
            return _min_history_len;
        }

    protected:
        void _remove_duplicates(AgentPose& l, int num_robots)
        {
            for (size_t i = 0; i < l.size(); ++i) {
                for (size_t j = 0; j < l.size(); ++j) {
                    if (i == j || i < num_robots) {
                        continue;
                    }

                    auto ii = l[i];
                    auto ij = l[j];
                    if (euc_distance(ii, ij) < MIN_DIST_PER_DUPL) {
                        l[i].pose.xyz.x = INVALID;
                        l[i].pose.xyz.y = INVALID;
                        l[i].pose.rpy.yaw = INVALID;
                    }
                }
            }

            std::vector<int> idcs;
            for (size_t i = 0; i < l.size(); ++i) {
                if (l[i].pose.xyz.x == INVALID) {
                    idcs.push_back(i);
                }
            }

            int removed = 0;
            for (auto idx : idcs) {
                l.erase(l.begin() + idx - removed);
            }
        }
        void _keep_more_distant(AgentPose& inds, AgentPose& robs, int num_agents, int num_robots)
        {
            auto rpose = robs[0];
            std::vector<size_t> idcs(inds.size() - 1); // minus the robot
            std::iota(idcs.begin(), idcs.end(), 1);
            std::stable_sort(idcs.begin(), idcs.end(), [&](size_t lhs, size_t rhs) {
                auto lind = inds[lhs];
                auto rind = inds[rhs];
                float ld = euc_distance(rpose, lind);
                float rd = euc_distance(rpose, rind);
                return ld < rd;
            });

            size_t removed = 0;
            size_t cidx = 1;
            size_t inds_sz = inds.size();
            while (inds_sz > num_agents) {
                inds.erase(inds.begin() + cidx - removed);
                ++cidx;
                --inds_sz;
            }
        }

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

        bool _force_robot_position;
        int _min_history_len;
    }; // namespace bobi
} // namespace bobi

#endif
