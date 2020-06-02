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

  // Figure out what the total segment time will be.
  double total_segment_distance = (goal.position_W - start.position_W).norm();
  // Total time needed to get to max speed (or go from max speed to 0).
  double min_acceleration_time = constraints_.v_max / constraints_.a_max;
  // The amount of distance covered during the acceleration (or decceleration
  // process).
  double min_acceleration_distance =
      constraints_.v_max * min_acceleration_time -
      0.5 * constraints_.a_max * min_acceleration_time * min_acceleration_time;

  double total_segment_time = 0.0;
  // Case 1: time is shorter than the acceleration and decceleration time.
  if (total_segment_distance < 2 * min_acceleration_distance) {
    total_segment_time =
        2 * std::sqrt(total_segment_distance / constraints_.a_max);
  } else {
    // Case 2: time is longer than accel + deccel time.
    total_segment_time =
        2 * min_acceleration_time +
        (total_segment_distance - 2 * min_acceleration_distance) /
            constraints_.v_max;
  }

  size_t num_elements = total_segment_time / constraints_.sampling_dt;

  path->clear();
  path->reserve(num_elements);

  Eigen::Vector3d path_direction =
      (goal.position_W - start.position_W).normalized();

  mav_msgs::EigenTrajectoryPoint point;
  point.position_W = start.position_W;
  point.orientation_W_B = goal.orientation_W_B;
  point.velocity_W = Eigen::Vector3d::Zero();
  point.time_from_start_ns = 0;
  int64_t dt_ns = mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

  if (verbose_) {
    ROS_INFO(
        "=== Ramp Statistics ==\n"
        "Total length [m]: %f\nTotal time [s]: %f\nNumber of samples: %lu\n"
        "Min accel dist [m]: %f\nMin accel time [s]: %f",
        total_segment_distance, total_segment_time, num_elements,
        min_acceleration_distance, min_acceleration_time);
  }

  // Treat this as a 1D problem since it is. ;)
  double position = 0;
  double velocity = 0;
  int64_t current_time = 0;

  for (size_t i = 0; i < num_elements; ++i) {
    // Integrate velocity to get position.
    position += velocity * constraints_.sampling_dt;

    // Figure out if we're accelerating, deccelerating, or neither.
    // Handle Case 1 first:
    if (total_segment_time < min_acceleration_time * 2) {
      if (current_time < total_segment_time / 2.0) {
        velocity += constraints_.a_max * constraints_.sampling_dt;
      } else {
        velocity -= constraints_.a_max * constraints_.sampling_dt;
      }
    } else {
      // Case 2
      if (position <= min_acceleration_distance) {
        velocity += constraints_.a_max * constraints_.sampling_dt;
      } else if ((total_segment_distance - position) <=
                 min_acceleration_distance) {
        velocity -= constraints_.a_max * constraints_.sampling_dt;
      }
    }

    // Make sure to meet constraints (could be passed/missed due to
    // discretization error).
    if (position > total_segment_distance) {
      position = total_segment_distance;
    }
    if (velocity > constraints_.v_max) {
      velocity = constraints_.v_max;
    }
    if (velocity < 0) {
      velocity = 0;
    }

    // Convert back to 3D.
    point.position_W = start.position_W + path_direction * position;
    point.velocity_W = path_direction * velocity;
    point.orientation_W_B = goal.orientation_W_B;
    point.time_from_start_ns = current_time;
    path->emplace_back(point);
    current_time += dt_ns;
  }
  point.position_W = goal.position_W;
  point.orientation_W_B = goal.orientation_W_B;
  point.velocity_W = Eigen::Vector3d::Zero();
  point.time_from_start_ns += dt_ns;
  path->emplace_back(point);

  return true;
}

}  // namespace mav_planning
