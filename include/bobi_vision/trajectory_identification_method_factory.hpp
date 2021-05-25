#ifndef BOBI_TRAJECTORY_IDENTIFICATION_METHOD_FACTORY_HPP
#define BOBI_TRAJECTORY_IDENTIFICATION_METHOD_FACTORY_HPP

#include <bobi_vision/TrajectoryIdentificationConfig.h>

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_vision/nearest_centroid.hpp>

namespace bobi {
    class TrajectoryIdentificationMethodFactory {
    public:
        std::pair<std::shared_ptr<FilteringMethodBase>, bool> operator()(const int choice)
        {
            std::shared_ptr<FilteringMethodBase> new_filter;
            bool success = true;

            switch (choice) {

            case bobi_vision::TrajectoryIdentification_None:
                new_filter = std::make_shared<FilteringMethodBase>(FilteringMethodBase());
                break;

            case bobi_vision::TrajectoryIdentification_NearestCentroids:
                new_filter = std::make_shared<NearestCentroid>(NearestCentroid());
                break;

                // case bobi_vision::TrajectoryIdentification_MotionModel:
                //     break;

                // case bobi_vision::TrajectoryIdentification_Kalman:
                //     break;

            default:
                new_filter = std::make_shared<FilteringMethodBase>(FilteringMethodBase());
                success = false;
                break;
            }

            return std::pair<std::shared_ptr<FilteringMethodBase>, bool>(new_filter, success);
        }
    };
} // namespace bobi

#endif