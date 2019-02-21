#ifndef MAV_PLANNING_COMMON_YAW_POLICY_H_
#define MAV_PLANNING_COMMON_YAW_POLICY_H_

#include <mav_msgs/eigen_mav_msgs.h>

#include "mav_planning_common/physical_constraints.h"

namespace mav_planning {

class YawPolicy {
 public:
  enum class PolicyType {
    kFromPlan = 0,
    kVelocityVector,
    kAnticipateVelocityVector,
    kPointFacing,
    kConstant
  };

  YawPolicy()
      : policy_(PolicyType::kFromPlan), sampling_dt_(-1), yaw_rate_max_(-1) {}
  YawPolicy(PolicyType policy)
      : policy_(policy), sampling_dt_(-1), yaw_rate_max_(-1) {}

  void setYawPolicy(PolicyType policy) { policy_ = policy; }
  PolicyType getYawPolicy() const { return policy_; }

  void setPhysicalConstraints(const PhysicalConstraints& constraints);

  void setSamplingDt(double sampling_dt);
  double getSamplingDt() { return sampling_dt_; }

  void setYawRateMax(double yaw_rate_max);
  double getYawRateMax() { return yaw_rate_max_; }

  void deactivateMaxYawRate() { yaw_rate_max_ = -1; }

  void applyPolicyInPlace(mav_msgs::EigenTrajectoryPointVector* path);
  void applyPolicy(const mav_msgs::EigenTrajectoryPointVector& path_in,
                   mav_msgs::EigenTrajectoryPointVector* path_out);

  void setFacingPoint(const Eigen::Vector3d& point) { facing_point_ = point; }
  Eigen::Vector3d getFacingPoint() const { return facing_point_; }

  void setConstantYaw(double yaw) { constant_yaw_ = yaw; }
  double getConstantYaw() const { return constant_yaw_; }

 private:
  PolicyType policy_;
  double sampling_dt_;
  double yaw_rate_max_;

  // Only used for constant yaw policy.
  double constant_yaw_;
  // Only used for point-facing policy.
  Eigen::Vector3d facing_point_;

  double getFeasibleYaw(double last_yaw, double desired_yaw);
};

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_YAW_POLICY_H_
