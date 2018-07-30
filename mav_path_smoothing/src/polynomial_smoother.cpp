#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

#include "mav_path_smoothing/polynomial_smoother.h"

namespace mav_planning {

bool PolynomialSmoother::getTrajectoryBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mav_trajectory_generation::Trajectory* trajectory) const {
  if (waypoints.size() < 2) {
    return false;
  }

  // I guess this is only if method is linear.
  mav_trajectory_generation::timing::Timer linear_timer(
      "smoothing/poly_linear");

  constexpr int N = 10;
  constexpr int K = 3;
  mav_trajectory_generation::PolynomialOptimization<N> poly_opt(K);

  int num_vertices = waypoints.size();

  int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::JERK;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_vertices, mav_trajectory_generation::Vertex(K));

  // Add the first and last.
  vertices.front().makeStartOrEnd(0, derivative_to_optimize);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.front().position_W);
  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.back().position_W);

  // Now do the middle bits.
  size_t j = 1;
  for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
    vertices[j].addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].position_W);
    j++;
  }

  std::vector<double> segment_times;
  mav_trajectory_generation::estimateSegmentTimesVelocityRamp(
      vertices, constraints_.v_max, constraints_.a_max, 1, &segment_times);

  poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  if (poly_opt.solveLinear()) {
    poly_opt.getTrajectory(trajectory);
    return true;
  }
  return false;
}

bool PolynomialSmoother::getPathBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPointVector& waypoints,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  mav_trajectory_generation::Trajectory trajectory;
  bool success = getTrajectoryBetweenWaypoints(waypoints, &trajectory);
  if (success) {
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, path);
    return true;
  }
  return false;
}

bool PolynomialSmoother::getPathBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);

  return getPathBetweenWaypoints(waypoints, path);
}

}  // namespace mav_planning
