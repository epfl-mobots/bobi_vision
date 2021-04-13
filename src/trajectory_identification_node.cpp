#include <ros/ros.h>

#include <bobi_vision/trajectory_identification.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_identification_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    TrajectoryIdentification ti(nh);

    ros::Publisher pose_pub = nh->advertise<bobi_msgs::PoseVec>("filtered_poses", 1);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        std::vector<bobi_msgs::PoseStamped> filtered = ti.filter();
        bobi_msgs::PoseVec pv;
        pv.poses = filtered;
        pose_pub.publish(pv);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}