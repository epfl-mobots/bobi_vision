#ifndef BOBI_FILTERING_METHOD_BASE_HPP
#define BOBI_FILTERING_METHOD_BASE_HPP

#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/ConvertCoordinates.h>

namespace bobi {
    class FilteringMethodBase {
    public:
        using AgentPose = std::vector<bobi_msgs::PoseStamped>;
        using AgentPoseList = std::list<AgentPose>;

        FilteringMethodBase(bool force_robot_position = true) : _force_robot_position(force_robot_position)
        {
            std::srand(time(NULL));
        }

        virtual void operator()(
            AgentPoseList& individual_poses,
            AgentPoseList& robot_poses,
            size_t num_agents,
            size_t num_robots)
        {
        }

    protected:
        bool _force_robot_position;
    };
} // namespace bobi

#endif
