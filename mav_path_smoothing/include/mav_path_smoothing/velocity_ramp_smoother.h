#ifndef MAV_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_
#define MAV_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_

#include "mav_path_smoothing/path_smoother_base.h"

namespace mav_planning {

class VelocityRampSmoother : public PathSmootherBase {
 public:
  VelocityRampSmoother() : PathSmootherBase() {}
  virtual ~VelocityRampSmoother {}

  virtual void setParametersFromRos(const ros::NodeHandle& nh);

  virtual bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPointVector* path) const;

 protected:
  bool verbose_;
};

}  // namespace mav_planning

#endif  // MAV_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_
