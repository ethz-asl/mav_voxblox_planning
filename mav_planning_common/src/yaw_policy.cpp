#include <ros/console.h>

#include "mav_planning_common/yaw_policy.h"

namespace mav_planning {

void YawPolicy::setPhysicalConstraints(const PhysicalConstraints& constraints) {
  sampling_dt_ = constraints.sampling_dt;
  yaw_rate_max_ = constraints.yaw_rate_max;
}

void YawPolicy::setSamplingDt(double sampling_dt) {
  if (sampling_dt <= 0.0) {
    ROS_ERROR("Sampling dt must be non-zero and positive!");
    return;
  }
  sampling_dt_ = sampling_dt;
}

void YawPolicy::setYawRateMax(double yaw_rate_max) {
  if (yaw_rate_max <= 0.0) {
    ROS_ERROR("Max yaw rate must be positive!");
    return;
  }
  yaw_rate_max_ = yaw_rate_max;
}

void YawPolicy::applyPolicyInPlace(mav_msgs::EigenTrajectoryPointVector* path) {
  double initial_yaw = 0.0;
  bool initial_yaw_given = false;
  applyPolicyInPlace(initial_yaw, path, initial_yaw_given);
}

void YawPolicy::applyPolicy(const mav_msgs::EigenTrajectoryPointVector& path_in,
                            mav_msgs::EigenTrajectoryPointVector* path_out) {
  double initial_yaw = 0.0;
  bool initial_yaw_given = false;
  applyPolicy(path_in, initial_yaw, path_out, initial_yaw_given);
}

void YawPolicy::applyPolicyInPlace(double initial_yaw,
                                   mav_msgs::EigenTrajectoryPointVector* path,
                                   bool initial_yaw_given) {
  // To check feasibility of next yaw
  double last_yaw = initial_yaw_given ? initial_yaw : constant_yaw_;
  // To continue tracking a yaw if it has not been reached yet and no new yaw
  // has to be tracked.
  double last_desired_yaw = last_yaw;
  bool valid_last_yaw = initial_yaw_given;

  if (policy_ == PolicyType::kFromPlan) {
    // Don't have to do anything! :)
    return;
  } else if (policy_ == PolicyType::kVelocityVector) {
    // Track velocity vector. If current point has near-zero velocity, track
    // next non-zero velocity. If there is none (end of trajectory), track last
    // non-zero velocity.
    const double kMinVelocityNorm = 0.1;
    for (size_t i = 0; i < path->size(); ++i) {
      // Non-const ref that gets modified below.
      mav_msgs::EigenTrajectoryPoint& point = (*path)[i];

      // Case 1: non-zero velocity at current point (ignoring vertical
      // component).
      Eigen::Vector3d velocity_xy = point.velocity_W;
      velocity_xy.z() = 0;
      if (velocity_xy.norm() > kMinVelocityNorm) {
        double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
        if (!valid_last_yaw) {
          // No initial yaw had been chosen yet
          last_yaw = desired_yaw;
        }
        double yaw = getFeasibleYaw(last_yaw, desired_yaw);
        point.setFromYaw(yaw);
        last_yaw = yaw;
        last_desired_yaw = desired_yaw;
        valid_last_yaw = true;
      } else {
        // Case 2: near-zero velocity.
        double desired_yaw;
        size_t j = i + 1;
        while (j < path->size() && velocity_xy.norm() < kMinVelocityNorm) {
          velocity_xy = (*path)[j].velocity_W;
          velocity_xy.z() = 0;
          j++;
        }
        if (velocity_xy.norm() > kMinVelocityNorm) {
          // Case 2a: there is a non-zero velocity point further on
          desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
        } else {
          // Case 2b: no non-zero velocity point further on, track last valid
          // one
          if (!valid_last_yaw) {
            ROS_ERROR(
                "No valid velocity found in trajectory and no initial yaw "
                "given! Setting yaw to 0.");
            desired_yaw = 0;
            valid_last_yaw = true;  // Such that error is thrown only once per
                                    // trajectory, on the rest of trajectory yaw
                                    // will anyways be 0
          } else {
            desired_yaw = last_desired_yaw;
          }
        }
        double yaw = getFeasibleYaw(last_yaw, desired_yaw);
        point.setFromYaw(yaw);
        last_yaw = yaw;
        last_desired_yaw = desired_yaw;
      }
    }
  } else if (policy_ == PolicyType::kAnticipateVelocityVector) {
    // Same as kVelocityVector, but max_yaw_rate is guaranteed by anticipating
    // the future desired yaws.
    valid_last_yaw = false;
    for (size_t i = path->size(); i > 0; --i) {
      // Non-const ref that gets modified below.
      mav_msgs::EigenTrajectoryPoint& point = (*path)[i - 1];
      const double kMinVelocityNorm = 0.1;

      // Case 1: non-zero velocity at current point (ignoring vertical
      // component).
      Eigen::Vector3d velocity_xy = point.velocity_W;
      velocity_xy.z() = 0;
      if (velocity_xy.norm() > kMinVelocityNorm) {
        double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
        if (!valid_last_yaw) {
          // No initial yaw had been chosen yet
          last_yaw = desired_yaw;
        }
        double yaw = getFeasibleYaw(last_yaw, desired_yaw);
        point.setFromYaw(yaw);
        last_yaw = yaw;
        last_desired_yaw = desired_yaw;
        valid_last_yaw = true;
      } else {
        // Case 2: near-zero velocity.
        double desired_yaw;
        for (size_t j = i; j > 0 && velocity_xy.norm() < kMinVelocityNorm;
             --j) {
          velocity_xy = (*path)[j - 1].velocity_W;
          velocity_xy.z() = 0;
        }
        if (velocity_xy.norm() > kMinVelocityNorm) {
          // Case 2a: there is a non-zero velocity point further on
          desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
        } else {
          // Case 2b: no non-zero velocity point further on, track last valid
          // one
          if (!valid_last_yaw) {
            if (initial_yaw_given) {
              ROS_WARN(
                  "No valid velocity found in trajectory, setting yaw to "
                  "initial_yaw: %f",
                  initial_yaw);
              desired_yaw = initial_yaw;
            } else {
              ROS_ERROR(
                  "No valid velocity found in trajectory and no initial yaw "
                  "given! Setting yaw to 0.");
              desired_yaw = 0;
            }
            valid_last_yaw = true;
          } else {
            desired_yaw = last_desired_yaw;
          }
        }
        double yaw = getFeasibleYaw(last_yaw, desired_yaw);
        point.setFromYaw(yaw);
        last_yaw = yaw;
        last_desired_yaw = desired_yaw;
      }
    }

    if (initial_yaw_given && path->size() > 0) {
      const double kYawDifferenceThreshold = 0.01;
      // Now set initial part of the trajectory so that initial_yaw is achieved.
      (*path)[0].setFromYaw(initial_yaw);
      last_yaw = initial_yaw;
      for (size_t i = 0; i < path->size(); ++i) {
        // Non-const ref that gets modified below.
        mav_msgs::EigenTrajectoryPoint& point = (*path)[i];
        double desired_yaw = point.getYaw();
        double yaw = getFeasibleYaw(last_yaw, desired_yaw);
        if (yaw - desired_yaw < kYawDifferenceThreshold) {
          // Now just follow yaw that was computed before.
          break;
        }
        point.setFromYaw(yaw);
        last_yaw = yaw;
      }
    }
  } else if (policy_ == PolicyType::kPointFacing) {
    // Try to figure out how to constantly face toward the same point.
    for (size_t i = 0; i < path->size(); ++i) {
      // Non-const ref that gets modified below.
      mav_msgs::EigenTrajectoryPoint& point = (*path)[i];
      Eigen::Vector3d facing_direction = facing_point_ - point.position_W;

      // Catch the case of both inputs being 0... When we pass directly over
      // the facing point.
      double desired_yaw = last_desired_yaw;
      if (std::abs(facing_direction.x()) > 1e-4 ||
          std::abs(facing_direction.y()) > 1e-4) {
        desired_yaw = atan2(facing_direction.y(), facing_direction.x());
      }
      if (!valid_last_yaw) {
        last_yaw = desired_yaw;
        valid_last_yaw = true;
      }
      double yaw = getFeasibleYaw(last_yaw, desired_yaw);
      point.setFromYaw(yaw);
      last_yaw = yaw;
      last_desired_yaw = desired_yaw;
    }
  } else if (policy_ == PolicyType::kConstant) {
    // Just go through the whole path and update the whole thing.
    if (!initial_yaw_given) {
      last_yaw = constant_yaw_;
    }
    for (size_t i = 0; i < path->size(); ++i) {
      // Non-const ref that gets modified below.
      mav_msgs::EigenTrajectoryPoint& point = (*path)[i];
      double yaw = getFeasibleYaw(last_yaw, constant_yaw_);
      point.setFromYaw(yaw);
      last_yaw = yaw;
    }
  }
}

void YawPolicy::applyPolicy(const mav_msgs::EigenTrajectoryPointVector& path_in,
                            double initial_yaw,
                            mav_msgs::EigenTrajectoryPointVector* path_out,
                            bool initial_yaw_given) {
  path_out->clear();
  *path_out = path_in;

  applyPolicyInPlace(initial_yaw, path_out, initial_yaw_given);
}

double YawPolicy::getFeasibleYaw(double last_yaw, double desired_yaw) {
  if (yaw_rate_max_ < 0) {  // yaw_rate_max_ deactivated
    return desired_yaw;
  }
  if (sampling_dt_ < 0) {
    std::cerr << "sampling_dt_ has to be set!\n";
    return 0;
  }
  // Compute delta yaw (between -PI and PI)
  double yaw_mod = fmod(desired_yaw - last_yaw, 2 * M_PI);
  if (yaw_mod < -M_PI) {
    yaw_mod += 2 * M_PI;
  } else if (yaw_mod > M_PI) {
    yaw_mod -= 2 * M_PI;
  }

  // Check if yaw rate is feasible
  if (std::abs(yaw_mod) > yaw_rate_max_ * sampling_dt_) {
    double yaw_direction = yaw_mod > 0.0 ? 1.0 : -1.0;
    return last_yaw + yaw_direction * yaw_rate_max_ * sampling_dt_;
  }
  return desired_yaw;
}

}  // namespace mav_planning
