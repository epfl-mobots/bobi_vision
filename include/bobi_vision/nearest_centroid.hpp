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
        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
            if (individual_poses.size() <= 1 || individual_poses.begin()->size() == 0) {
                return;
            }

            std::vector<int> idcs(num_agents);
            std::iota(idcs.begin(), idcs.end(), 0);

            auto euc_distance = [](const bobi_msgs::Pose& n, const bobi_msgs::Pose& o) {
                return std::sqrt(
                    std::pow(o.xyz.x - n.xyz.x, 2.)
                    + std::pow(o.xyz.y - n.xyz.y, 2.));
            };

            auto angle_diff = [&](const bobi_msgs::Pose& n, const bobi_msgs::Pose& o) {
                return _angle_to_pipi(n.rpy.yaw - o.rpy.yaw);
            };

            // { // assign individual ids according to the robot positions
            //     AgentPoseList::iterator cur_robot = robot_poses.begin();
            //     AgentPoseList::iterator cur_ind = individual_poses.begin();
            //     for (size_t i = 0; i < num_robots; ++i) {
            //         bobi_msgs::Pose robot_pose = (*cur_robot)[i].pose;
            //         float min_dist = euc_distance((*cur_ind)[0].pose, robot_pose);
            //         int idx = 0;
            //         for (size_t j = 1; j < (*cur_ind).size(); ++j) {
            //             float dist = euc_distance((*cur_ind)[j].pose, robot_pose);
            //             if (dist < min_dist) {
            //                 min_dist = dist;
            //                 idx = j;
            //             }
            //         }
            //         bobi_msgs::PoseStamped tmp = (*cur_ind)[i];
            //         (*cur_ind)[i] = (*cur_ind)[idx];
            //         (*cur_ind)[idx] = tmp;
            //         idcs.erase(std::remove(idcs.begin(), idcs.end(), i), idcs.end());
            //     }
            // }

            AgentPoseList::iterator cur = individual_poses.begin();
            AgentPoseList::iterator prev = individual_poses.begin();
            std::advance(prev, 1);

            if ((*cur).size() == 0) {
                return;
            }

            std::cout << "ok1" << std::endl;
            std::vector<bobi_msgs::PoseStamped> ind_poses_cpy(*cur);
            for (size_t i = 0; i < (*cur).size(); ++i) {
                float min_dist = euc_distance((*cur)[i].pose, (*prev)[idcs[0]].pose);
                float min_angle_diff = abs(angle_diff((*cur)[i].pose, (*prev)[idcs[0]].pose)) / M_PI;
                float min_val = (min_dist * min_angle_diff)
                    / (std::pow(min_dist, 2.), std::pow(min_angle_diff, 2.));
                int idx = idcs[0];

                for (size_t j = 0; j < (*cur).size(); ++j) {
                    float dist = euc_distance((*cur)[i].pose, (*prev)[j].pose);
                    float ad = abs(angle_diff((*cur)[i].pose, (*prev)[j].pose)) / M_PI;
                    float val = (dist * ad)
                        / (std::pow(dist, 2.) * std::pow(ad, 2.));

                    if (val < min_val && std::find(idcs.begin(), idcs.end(), j) != idcs.end()) {
                        min_val = val;
                        idx = j;
                    }
                }
                std::cout << "ok2" << std::endl;
                bobi_msgs::PoseStamped tmp = ind_poses_cpy[idx];
                ind_poses_cpy[idx] = ind_poses_cpy[i];
                ind_poses_cpy[i] = tmp;
                std::cout << "ok3" << std::endl;

                idcs.erase(std::remove(idcs.begin(), idcs.end(), idx), idcs.end());
                std::cout << "ok4" << std::endl;
            }

            std::cout << "ok5" << std::endl;

            for (const int idx : idcs) {
                std::cout << ind_poses_cpy.size() << " " << idx << " " << prev->size() << std::endl;
                ind_poses_cpy.insert(ind_poses_cpy.begin() + idx, (*prev)[idx]);
            }
            std::cout << "ok6" << std::endl;

            *cur = ind_poses_cpy;
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
    };
} // namespace bobi

#endif
