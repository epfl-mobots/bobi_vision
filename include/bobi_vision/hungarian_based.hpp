#ifndef BOBI_HUNGARIAN_BASED_HPP
#define BOBI_HUNGARIAN_BASED_HPP

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_msgs/Pose.h>

#include <numeric>
#include <algorithm>

#include <bobi_vision/hungarian_algorithm.hpp>
#include <bobi_vision/nearest_centroid.hpp>

#include <opencv2/opencv.hpp>

#define MAX_BL_IN_PX 140

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
            size_t sidx = num_robots;

            if (num_robots > 0 && r0->size()) {
                for (size_t i = 0; i < r0->size(); ++i) {
                    float min_dist = std::numeric_limits<float>::infinity();
                    int min_idx = INVALID;
                    for (size_t j = 0; j < t0->size(); ++j) {
                        float dist = euc_distance((*r0)[i], (*t0)[i]);
                        if (dist < min_dist) {
                            min_dist = dist;
                            min_idx = j;
                        }
                    }

                    if (min_idx != INVALID && min_idx != i) {
                        bobi_msgs::PoseStamped tmp;
                        if (!_force_robot_position) {
                            copy_pose((*t0)[min_idx], tmp);
                            copy_pose((*t0)[i], (*t0)[min_idx]);
                            copy_pose(tmp, (*t0)[i]);
                        }
                        else {
                            copy_pose((*r0)[i], (*t0)[i]);
                            (*t0)[i].pose.is_swapped = true;
                        }
                    }
                }
            }

            AgentPose copy;
            if (t0->size()) {
                copy.resize(t0->size());
                for (size_t i = 0; i < copy.size(); ++i) {
                    copy_pose((*t0)[i], copy[i]);
                }
                // std::copy(t0->begin(), t0->end(), copy.begin());
            }
            else {
                if (num_robots && r0->size()) {
                    copy.resize(r0->size());
                    t0->resize(r0->size());
                    std::copy(r0->begin(), r0->end(), copy.begin());
                    std::copy(r0->begin(), r0->end(), t0->begin());
                }
            }

            if (copy.size() < num_agents) {
                std::vector<std::vector<cv::Point>> contours;
                for (const bobi_msgs::PoseStamped& pose : *t0) {
                    std::vector<cv::Point> c;
                    for (auto point : pose.pose.contours) {
                        cv::Point p;
                        p.x = point.x;
                        p.y = point.y;
                        c.push_back(p);
                    }
                    contours.push_back(c);
                }

                _split_big_contours(copy, *t0, contours, 0, false);
                t0->resize(copy.size());
                for (size_t i = 0; i < copy.size(); ++i) {
                    copy_pose(copy[i], (*t0)[i]);
                }
                // std::copy(copy.begin(), copy.end(), std::back_inserter(*t0));
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
                cost_mat.resize(copy.size() - sidx);
                for (size_t i = sidx; i < copy.size(); ++i) {
                    cost_mat[i].resize(t1->size() - sidx);
                    for (size_t j = sidx; j < t1->size(); ++j) {
                        double dist = euc_distance(copy[i], (*t1)[j]);
                        double asim = angle_sim(copy[i], (*t1)[j]);

                        double w_dist = 1.0;
                        double w_hdg = 0.0;
                        double cost = w_dist * dist + w_hdg * asim;
                        cost_mat[i - sidx][j - sidx] = cost;
                    }
                }

                std::vector<int> assignment_idcs;
                _ha.solve(cost_mat, assignment_idcs);
                std::vector<size_t> not_matched;
                t0->clear();

                for (size_t i = 0; i < cost_mat.size(); ++i) {
                    bobi_msgs::PoseStamped p;
                    copy_pose(copy[i + sidx], p);
                    if (assignment_idcs[i] >= 0) {
                        t0->push_back(p);
                    }
                    else {
                        not_matched.push_back(i + sidx);
                    }
                }

                for (size_t i = 0; i < not_matched.size(); ++i) {
                    bobi_msgs::PoseStamped p;
                    copy_pose(copy[not_matched[i]], p);
                    t0->push_back(p);
                }
            }

            // if (num_robots > 0 && r0->size()) {
            //     for (size_t i = 0; i < r0->size(); ++i) {
            //         float min_dist = std::numeric_limits<float>::infinity();
            //         int min_idx = INVALID;
            //         for (size_t j = 0; j < t0->size(); ++j) {
            //             float dist = euc_distance((*r0)[i], (*t0)[i]);
            //             if (dist < min_dist) {
            //                 min_dist = dist;
            //                 min_idx = j;
            //             }
            //         }

            //         if (min_idx != INVALID && min_idx != i) {
            //             bobi_msgs::PoseStamped tmp;
            //             if (!_force_robot_position) {
            //                 copy_pose((*t0)[min_idx], tmp);
            //                 copy_pose((*t0)[i], (*t0)[min_idx]);
            //                 copy_pose(tmp, (*t0)[i]);
            //             }
            //             else {
            //                 copy_pose((*r0)[i], (*t0)[i]);
            //                 (*t0)[i].pose.is_swapped = true;
            //             }
            //         }
            //     }
            // }
        }

    protected:
        algo::HungarianAlgorithm _ha;

        void copy_pose(const bobi_msgs::PoseStamped& in, bobi_msgs::PoseStamped& target)
        {
            target.pose.xyz = in.pose.xyz;
            target.pose.rpy = in.pose.rpy;
            std::copy(in.pose.contours.begin(), in.pose.contours.end(), std::back_inserter(target.pose.contours));
            target.pose.is_filtered = in.pose.is_filtered;
            target.pose.is_swapped = in.pose.is_swapped;
        }

        void _split_big_contours(AgentPose& copy, const AgentPose& l, const std::vector<std::vector<cv::Point>>& contours, int sidx, bool is_robot = true)
        {
            size_t added = 0;
            for (size_t i = sidx; i < l.size(); ++i) {
                float area = cv::contourArea(contours[i]);
                if (area > MAX_BL_IN_PX) {
                    int dubs = static_cast<int>(area / MAX_BL_IN_PX);
                    for (int j = 0; j < dubs; ++j) {
                        copy.insert(copy.begin() + i + added, l[i]);
                        ++added;
                    }
                }
            }
        }
    };
} // namespace bobi

#endif
