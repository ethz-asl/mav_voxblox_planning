#include <mav_trajectory_generation/vertex.h>

#include "mav_planning_common/visibility_resampling.h"

namespace mav_planning {

void resampleWaypointsFromVisibilityGraph(
    int num_segments, const PhysicalConstraints& constraints,
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_msgs::EigenTrajectoryPoint::Vector* waypoints_out) {
  // We will divide the trajectory into num_segments+1 waypoints.
  CHECK_NOTNULL(waypoints_out);
  CHECK_GT(waypoints.size(), 1u);
  waypoints_out->reserve(num_segments + 1);
  // Create a temporary estimate of segment times for the original waypoints,
  // for convenience.
  std::vector<double> segment_times;
  segment_times.reserve(waypoints.size() - 1);

  for (size_t i = 1; i < waypoints.size(); i++) {
    segment_times.push_back(mav_trajectory_generation::computeTimeVelocityRamp(
        waypoints[i - 1].position_W, waypoints[i].position_W, constraints.v_max,
        constraints.a_max));
  }

  double total_time =
      std::accumulate(segment_times.begin(), segment_times.end(), 0.0);
  ROS_INFO("Total time: %f Time per seg: %f", total_time,
           total_time / num_segments);
  // Next we'll split the total time into num_segments sections and evaluate
  // where the waypoint falls.
  double time_so_far = 0.0;
  double time_per_segment = total_time / num_segments;
  size_t input_waypoint_index = 0;
  size_t output_waypoint_index = 1;

  waypoints_out->push_back(waypoints.front());

  while (output_waypoint_index < num_segments) {
    if (time_so_far >= time_per_segment * output_waypoint_index) {
      mav_msgs::EigenTrajectoryPoint point;
      Eigen::Vector3d direction =
          waypoints[input_waypoint_index].position_W -
          waypoints[input_waypoint_index - 1].position_W;
      double magnitude =
          1.0 -
          (time_so_far - time_per_segment * output_waypoint_index) /
              segment_times[input_waypoint_index - 1];
      point.position_W = waypoints[input_waypoint_index - 1].position_W +
                         magnitude * direction;
      waypoints_out->push_back(point);
      ROS_INFO("Waypoint %d from waypoint %d at time: %f offset: %f",
               output_waypoint_index, input_waypoint_index, time_so_far,
               magnitude);
      output_waypoint_index++;
    } else {
      time_so_far += segment_times[input_waypoint_index];
      input_waypoint_index++;
    }
  }
  waypoints_out->push_back(waypoints.back());
}

}  // namespace mav_planning
