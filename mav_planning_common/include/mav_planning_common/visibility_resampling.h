#ifndef MAV_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_
#define MAV_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_

#include <mav_msgs/eigen_mav_msgs.h>

#include "mav_planning_common/physical_constraints.h"


namespace mav_planning {

void resampleWaypointsFromVisibilityGraph(
    int num_segments, const PhysicalConstraints& constraints,
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_msgs::EigenTrajectoryPoint::Vector* waypoints_out);

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_
