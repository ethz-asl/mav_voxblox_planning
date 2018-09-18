#include <loco_planner/loco.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_path_smoothing/loco_smoother.h"

namespace mav_planning {

LocoSmoother::LocoSmoother()
    : PolynomialSmoother(),
      resample_trajectory_(false),
      num_segments_(3),
      add_waypoints_(false) {
  split_at_collisions_ = false;
}

void LocoSmoother::setParametersFromRos(const ros::NodeHandle& nh) {
  PolynomialSmoother::setParametersFromRos(nh);
  nh.param("resample_trajectory", resample_trajectory_, resample_trajectory_);
  nh.param("add_waypoints", add_waypoints_, add_waypoints_);
  nh.param("num_segments", num_segments_,
           num_segments_);

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
  PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints, &traj_initial);

  constexpr int N = 10;
  constexpr int D = 3;
  loco_planner::Loco<N> loco(D);
  // This is because our initial solution is nearly collision-free.
  loco.setWd(10.0);

  loco.setRobotRadius(constraints_.robot_radius);
  loco.setMapResolution(min_col_check_resolution_);
  loco.setDistanceFunction(map_distance_func_);

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

  return true;

  mav_msgs::EigenTrajectoryPoint::Vector path;
  // Sample it!
  double dt = constraints_.sampling_dt;
  mav_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);

  double t = 0.0;
  bool path_in_collision = isPathInCollision(path, &t);
  return !path_in_collision;
}

bool LocoSmoother::getTrajectoryBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_trajectory_generation::Trajectory* trajectory) const {
  constexpr int N = 10;
  constexpr int D = 3;
  loco_planner::Loco<N> loco(D);
  loco.setWd(0.1);

  loco.setRobotRadius(constraints_.robot_radius);
  loco.setMapResolution(min_col_check_resolution_);
  loco.setDistanceFunction(map_distance_func_);

  double total_time = mav_trajectory_generation::computeTimeVelocityRamp(
      start.position_W, goal.position_W, constraints_.v_max,
      constraints_.a_max);
  loco.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);
  loco.solveProblem();
  loco.getTrajectory(trajectory);

  return true;
}

bool LocoSmoother::getPathBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  mav_trajectory_generation::Trajectory trajectory;
  bool success = getTrajectoryBetweenTwoPoints(start, goal, &trajectory);
  if (success) {
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, path);
    return true;
  }
  return false;
}

}  // namespace mav_planning
