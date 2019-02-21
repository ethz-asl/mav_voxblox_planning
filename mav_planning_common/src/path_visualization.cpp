#include <mav_msgs/conversions.h>

#include "mav_planning_common/path_visualization.h"

namespace mav_planning {

visualization_msgs::Marker createMarkerForPath(
    mav_msgs::EigenTrajectoryPointVector& path, const std::string& frame_id,
    const std_msgs::ColorRGBA& color, const std::string& name, double scale) {
  visualization_msgs::Marker path_marker;

  const int kMaxSamples = 1000;
  const int num_samples = path.size();
  int subsample = 1;
  while (num_samples / subsample > kMaxSamples) {
    subsample *= 10;
  }
  const double kMaxMagnitude = 1.0e4;

  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size() / subsample);
  int i = 0;
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    i++;
    if (i % subsample != 0) {
      continue;
    }
    // Check that we're in some reasonable bounds.
    // Makes rviz stop crashing.
    if (point.position_W.maxCoeff() > kMaxMagnitude ||
        point.position_W.minCoeff() < -kMaxMagnitude) {
      continue;
    }

    geometry_msgs::Point point_msg;
    mav_msgs::pointEigenToMsg(point.position_W, &point_msg);
    path_marker.points.push_back(point_msg);
  }

  return path_marker;
}

visualization_msgs::Marker createMarkerForWaypoints(
    mav_msgs::EigenTrajectoryPointVector& path, const std::string& frame_id,
    const std_msgs::ColorRGBA& color, const std::string& name, double scale) {
  visualization_msgs::Marker path_marker;

  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size());
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    geometry_msgs::Point point_msg;
    mav_msgs::pointEigenToMsg(point.position_W, &point_msg);
    path_marker.points.push_back(point_msg);
  }

  return path_marker;
}

}  // namespace mav_planning
