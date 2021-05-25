#ifndef BOBI_FILTERING_METHOD_BASE_HPP
#define BOBI_FILTERING_METHOD_BASE_HPP

#include <bobi_msgs/PoseStamped.h>

namespace bobi {
    class FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        FilteringMethodBase()
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
    };
} // namespace bobi

#endif
