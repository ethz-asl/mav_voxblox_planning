#include "mav_path_smoothing/velocity_ramp_smoother.h"

namespace mav_planning {

void PathSmootherBase::setParametersFromRos(const ros::NodeHandle& nh) {
  constraints_.setParametersFromRos(nh);
}

void PathSmootherBase::setPhysicalConstraints(const PhysicalConstraints& constraints) {
  constraints_ = constraints;
}
const PhysicalConstraints& PathSmootherBase::getPhysicalConstraints() const {
  return constraints_;
}

}  // namespace mav_planning
