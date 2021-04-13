#ifndef FILTERING_METHOD_BASE_HPP
#define FILTERING_METHOD_BASE_HPP

#include <bobi_msgs/PoseStamped.h>

namespace bobi {
    class FilteringMethodBase {

        using AgentPoseList = std::list<std::vector<bobi_msgs::PoseStamped>>;

    public:
        virtual void operator()(AgentPoseList& individual_poses, AgentPoseList& robot_poses)
        {
        }
    };
} // namespace bobi

#endif
