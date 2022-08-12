#include <ros/ros.h>

#include <bobi_vision/trajectory_identification.hpp>

#include <bobi_msgs/SpeedEstimateVec.h>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_identification_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    TrajectoryIdentification ti(nh, 15);

    ros::Publisher pose_pub = nh->advertise<bobi_msgs::PoseVec>("filtered_poses", 1);
    ros::Publisher speed_pub = nh->advertise<bobi_msgs::SpeedEstimateVec>("speed_estimates", 1);

    int rate = 30;
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        std::vector<bobi_msgs::PoseStamped> filtered = ti.filter();

        bobi_msgs::PoseVec pv;
        pv.poses = filtered;
        pose_pub.publish(pv);

        auto history = ti.filtered_list();
        if (history.size() > 1) {
            bobi_msgs::SpeedEstimateVec sev;
            auto filtered_t_1 = *std::next(history.begin());
            for (size_t i = 0; i < filtered.size(); ++i) {
                double dt = filtered[i].header.stamp.toSec() - filtered_t_1[i].header.stamp.toSec();
                dt = std::max(dt, 1. / rate);
                double speed = std::sqrt(std::pow(filtered[i].pose.xyz.x - filtered_t_1[i].pose.xyz.x, 2) + std::pow(filtered[i].pose.xyz.y - filtered_t_1[i].pose.xyz.y, 2)) / dt;
                sev.speeds.push_back(speed);
            }
            speed_pub.publish(sev);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}