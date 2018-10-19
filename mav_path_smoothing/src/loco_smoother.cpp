#include <loco_planner/loco.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_path_smoothing/loco_smoother.h"

namespace mav_planning {

LocoSmoother::LocoSmoother()
    : PolynomialSmoother(),
      resample_trajectory_(false),
      resample_visibility_(false),
      num_segments_(3),
      add_waypoints_(false) {
  split_at_collisions_ = false;
}

void LocoSmoother::setParametersFromRos(const ros::NodeHandle& nh) {
  PolynomialSmoother::setParametersFromRos(nh);
  nh.param("resample_trajectory", resample_trajectory_, resample_trajectory_);
  nh.param("resample_visbility", resample_visibility_, resample_visibility_);
  nh.param("add_waypoints", add_waypoints_, add_waypoints_);
  nh.param("num_segments", num_segments_, num_segments_);

  // Force some settings.
  split_at_collisions_ = false;
}

bool LocoSmoother::getTrajectoryBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_trajectory_generation::Trajectory* trajectory) const {
  // If there's less than 3 waypoints, there are no free variables for loco.
  if (waypoints.size() < 2) {
    return false;
  }
  if (waypoints.size() == 2) {
    return PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints,
                                                             trajectory);
  }
  mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

  // Create a loco object! So Loco!
  mav_trajectory_generation::Trajectory traj_initial;
  if (resample_visibility_) {
    // If resampling the visibility graph, then basically divide the whole thing
    // into evenly spaced waypoints on the graph.
    mav_msgs::EigenTrajectoryPoint::Vector resampled_waypoints;
    resampleWaypointsFromVisibilityGraph(waypoints, &resampled_waypoints);
    PolynomialSmoother::getTrajectoryBetweenWaypoints(resampled_waypoints,
                                                      &traj_initial);
  } else {
    PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints, &traj_initial);
  }

  constexpr int N = 10;
  constexpr int D = 3;
  loco_planner::Loco<N> loco(D);
  // This is because our initial solution is nearly collision-free.
  loco.setWd(10.0);

  loco.setRobotRadius(constraints_.robot_radius);
  loco.setMapResolution(min_col_check_resolution_);
  if (distance_and_gradient_function_) {
    loco.setDistanceAndGradientFunction(
        std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
  } else {
    loco.setDistanceFunction(map_distance_func_);
  }

  if (resample_trajectory_) {
    loco.setupFromTrajectoryAndResample(traj_initial, num_segments_);
  } else {
    loco.setupFromTrajectory(traj_initial);
  }
  if (add_waypoints_) {
    loco.setWaypointsFromTrajectory(traj_initial);
  }

  loco.solveProblem();
  loco.getTrajectory(trajectory);

  if (optimize_time_) {
    trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                   constraints_.a_max);
  }

  return true;
}

bool LocoSmoother::getTrajectoryBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_trajectory_generation::Trajectory* trajectory) const {
  mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

  CHECK_NOTNULL(trajectory);
  constexpr int N = 10;
  constexpr int D = 3;
  loco_planner::Loco<N> loco(D);
  loco.setWd(0.1);

  loco.setRobotRadius(constraints_.robot_radius);
  loco.setMapResolution(min_col_check_resolution_);
  if (distance_and_gradient_function_) {
    loco.setDistanceAndGradientFunction(
        std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
  } else {
    loco.setDistanceFunction(map_distance_func_);
  }

  double total_time = mav_trajectory_generation::computeTimeVelocityRamp(
      start.position_W, goal.position_W, constraints_.v_max,
      constraints_.a_max);
  loco.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);
  loco.solveProblem();
  loco.getTrajectory(trajectory);

  if (optimize_time_) {
    trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                   constraints_.a_max);
  }
  return true;
}

bool LocoSmoother::getPathBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  CHECK_NOTNULL(path);
  mav_trajectory_generation::Trajectory trajectory;
  bool success = getTrajectoryBetweenTwoPoints(start, goal, &trajectory);
  if (success) {
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, path);
    return true;
  }
  return false;
}

void LocoSmoother::resampleWaypointsFromVisibilityGraph(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_msgs::EigenTrajectoryPoint::Vector* waypoints_out) const {
  // We will divide the trajectory into num_segments_+1 waypoints.
  CHECK_NOTNULL(waypoints_out);
  CHECK_GT(waypoints.size(), 1u);
  waypoints_out->reserve(num_segments_ + 1);
  // Create a temporary estimate of segment times for the original waypoints,
  // for convenience.
  std::vector<double> segment_times;
  segment_times.reserve(waypoints.size() - 1);

  for (size_t i = 1; i < waypoints.size(); i++) {
    segment_times.push_back(mav_trajectory_generation::computeTimeVelocityRamp(
        waypoints[i - 1].position_W, waypoints[i].position_W,
        constraints_.v_max, constraints_.a_max));
  }

  double total_time =
      std::accumulate(segment_times.begin(), segment_times.end(), 0.0);

  // Next we'll split the total time into num_segments_ sections and evaluate
  // where the waypoint falls.
  double time_so_far = 0.0;
  double time_per_segment = total_time / num_segments_;
  size_t input_waypoint_index = 0;
  size_t output_waypoint_index = 1;

  waypoints_out->push_back(waypoints.front());

  while (time_so_far < total_time &&
         input_waypoint_index < segment_times.size()) {
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
      output_waypoint_index++;
    } else {
      time_so_far += segment_times[input_waypoint_index];
      input_waypoint_index++;
    }
  }
  waypoints_out->push_back(waypoints.back());
}

}  // namespace mav_planning
