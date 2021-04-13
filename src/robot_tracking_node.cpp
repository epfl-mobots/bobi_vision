#include <ros/ros.h>

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tracking_node");
    ros::NodeHandle nh;

    // pose publisher
    ros::Publisher pose_pub;
    pose_pub = nh.advertise<bobi_msgs::PoseVec>("robot_poses", 1);

    ros::Rate loop_rate(30); // TODO: sync with lowest fps value?
    while (ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        bobi_msgs::PoseVec pv;
        pose_pub.publish(pv);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}