#ifndef MAV_PATH_SMOOTHING_LOCO_SMOOTHER_H_
#define MAV_PATH_SMOOTHING_LOCO_SMOOTHER_H_

#include <loco_planner/loco.h>

#include "mav_path_smoothing/polynomial_smoother.h"

namespace mav_planning {

class LocoSmoother : public PolynomialSmoother {
 public:
  LocoSmoother();
  virtual ~LocoSmoother() {}

  virtual void setParametersFromRos(const ros::NodeHandle& nh);

  virtual bool getTrajectoryBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
      mav_trajectory_generation::Trajectory* trajectory) const;

 protected:
};

}  // namespace mav_planning

#endif  // MAV_PATH_SMOOTHING_LOCO_SMOOTHER_H_
