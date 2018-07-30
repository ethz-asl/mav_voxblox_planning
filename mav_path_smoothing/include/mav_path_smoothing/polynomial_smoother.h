#ifndef MAV_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_
#define MAV_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_

#include <mav_trajectory_generation/trajectory.h>

#include "mav_path_smoothing/path_smoother_base.h"

namespace mav_planning {

class PolynomialSmoother : public PathSmootherBase {
 public:
  PolynomialSmoother() : PathSmootherBase() {}
  virtual ~PolynomialSmoother() {}

  virtual void setParametersFromRos(const ros::NodeHandle& nh);

  virtual bool getTrajectoryBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
      mav_trajectory_generation::Trajectory* trajectory) const;

  virtual bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
      mav_msgs::EigenTrajectoryPoint::Vector* path) const;

  virtual bool getPathBetweenTwoPoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* path) const;

 protected:
  // Figure out what kind of polynomial smoothing to do...
  // Method maybe?
};

}  // namespace mav_planning

#endif  // MAV_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_
