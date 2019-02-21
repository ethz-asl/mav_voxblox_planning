#ifndef MAV_PLANNING_COMMON_PATH_VISUALIZATION_H_
#define MAV_PLANNING_COMMON_PATH_VISUALIZATION_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <visualization_msgs/Marker.h>

namespace mav_planning {

// Subsamples a trajectory path and visualizes it.
visualization_msgs::Marker createMarkerForPath(
    mav_msgs::EigenTrajectoryPointVector& path, const std::string& frame_id,
    const std_msgs::ColorRGBA& color, const std::string& name, double scale);

// Creates little spheres to visualize waypoints.
visualization_msgs::Marker createMarkerForWaypoints(
    mav_msgs::EigenTrajectoryPointVector& path, const std::string& frame_id,
    const std_msgs::ColorRGBA& color, const std::string& name, double scale);

}  // mav_planning

#endif  // MAV_PLANNING_COMMON_PATH_VISUALIZATION_H_
