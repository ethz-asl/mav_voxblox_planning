#include <mav_msgs/common.h>

#include "mav_path_smoothing/velocity_ramp_smoother.h"

namespace mav_planning {

void VelocityRampSmoother::setParametersFromRos(const ros::NodeHandle& nh) {
  PathSmootherBase::setParametersFromRos(nh);
}

bool VelocityRampSmoother::getPathBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* path) const {
  path->clear();

  if ((goal.position_W-start.position_W).norm() < 1e-6) {
    path->push_back(start);
    path->push_back(goal);
    return true;
  }

  Eigen::Vector3d path_direction =
      (goal.position_W - start.position_W).normalized();

  // get rotation matrix in direction of path
  const Eigen::Vector3d up(0.0, 0.0, 1.0);
  Eigen::Matrix3d R;
  R.row(2) = path_direction.normalized();            // direction
  R.row(0) = up.cross(R.row(2)).normalized();        // x-axis
  R.row(1) = R.row(2).cross(R.row(0)).normalized();  // y-axis

  // note path is planned in body frame and then rotated to world frame
  mav_msgs::EigenTrajectoryPoint point;
  point.position_W = R * start.position_W;
  point.orientation_W_B = goal.orientation_W_B;
  point.velocity_W = R * start.velocity_W;
  point.time_from_start_ns = 0;

  int64_t dt_ns = mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

  std::vector<bool> stopping = {false, false, false};

  // consider us stopped below this speed
  constexpr double kDeltaVelocity = 1e-6;

  // First put in the start pose.
  path->emplace_back(start);

  do {
    // Integrate velocity to get position.
    point.position_W += point.velocity_W * constraints_.sampling_dt;
    if (std::isnan(point.position_W.x())) {
      std::cout << "Next point is nan! Why? We don't know. Point: "
                << point.position_W.transpose()
                << " velocity: " << point.velocity_W.transpose() << std::endl;
    }
    // get offset from goal
    Eigen::Vector3d delta_goal = (R * goal.position_W) - point.position_W;

    for (size_t i = 0; i < 3; ++i) {
      if (!stopping[i]) {
        // try to match desired velocity
        double acc = std::copysign(1.0, delta_goal(i)) * constraints_.v_max -
                     point.velocity_W(i);
        point.velocity_W(i) += std::max(
            std::min(acc, constraints_.a_max * constraints_.sampling_dt),
            -constraints_.a_max * constraints_.sampling_dt);

        // if within min distance to stop before goal
        if (delta_goal(i) < (0.5 * point.velocity_W(i) * point.velocity_W(i) /
                             constraints_.a_max)) {
          stopping[i] = true;
        }

      } else {
        // stop as fast as possible
        point.velocity_W(i) -=
            std::copysign(1.0, point.velocity_W(i)) *
            std::min(std::abs(point.velocity_W(i)),
                     constraints_.a_max * constraints_.sampling_dt);
      }
    }

    // Update time.
    point.time_from_start_ns += dt_ns;

    // rotate back into world frame
    mav_msgs::EigenTrajectoryPoint point_W = point;
    point_W.position_W = R.inverse() * point.position_W;
    point_W.velocity_W = R.inverse() * point.velocity_W;
    path->emplace_back(point_W);

  } while (point.velocity_W.norm() > kDeltaVelocity);

  point.position_W = goal.position_W;
  point.orientation_W_B = goal.orientation_W_B;
  point.velocity_W = Eigen::Vector3d::Zero();
  path->emplace_back(point);

  return true;
}

}  // namespace mav_planning
