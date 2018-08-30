#include <loco_planner/loco.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_path_smoothing/loco_smoother.h"

namespace mav_planning {

LocoSmoother::LocoSmoother() : PolynomialSmoother() {
  split_at_collisions_ = false;
}

void LocoSmoother::setParametersFromRos(const ros::NodeHandle& nh) {
  PolynomialSmoother::setParametersFromRos(nh);
  // Force some settings.
  split_at_collisions_ = false;
}

bool LocoSmoother::getTrajectoryBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_trajectory_generation::Trajectory* trajectory) const {
  if (waypoints.size() < 2) {
    return false;
  }

  mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

  // Create a loco object! So Loco!
  mav_trajectory_generation::Trajectory traj_initial;
  PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints, &traj_initial);

  constexpr int N = 10;
  constexpr int D = 3;
  loco_planner::Loco<N> loco(D);

  // TODO(helenol): set other settings, like robot radius, etc.
  loco.setRobotRadius(constraints_.robot_radius);
  loco.setMapResolution(min_col_check_resolution_);

  loco.setDistanceFunction(map_distance_func_);

  loco.setupFromTrajectory(traj_initial);
  loco.solveProblem();
  loco.getTrajectory(trajectory);

  mav_msgs::EigenTrajectoryPoint::Vector path;

  // Sample it!
  double dt = constraints_.sampling_dt;
  mav_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);

  double t = 0.0;
  bool path_in_collision = isPathInCollision(path, &t);
  return path_in_collision;
}

}  // namespace mav_planning
