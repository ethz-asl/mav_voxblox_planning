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

  // Special case for num_waypoints = 2 (splits trajectory into num_segments).
  virtual bool getTrajectoryBetweenTwoPoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_trajectory_generation::Trajectory* trajectory) const;

  virtual bool getPathBetweenTwoPoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* path) const;


 protected:
  int num_segments_;
};

}  // namespace mav_planning

#endif  // MAV_PATH_SMOOTHING_LOCO_SMOOTHER_H_
