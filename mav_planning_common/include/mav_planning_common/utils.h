#ifndef MAV_PLANNING_COMMON_UTILS_H_
#define MAV_PLANNING_COMMON_UTILS_H_

#include <mav_msgs/eigen_mav_msgs.h>

namespace mav_planning {

double computePathLength(mav_msgs::EigenTrajectoryPointVector& path) {
  Eigen::Vector3d last_point;
  double distance = 0;
  for (int i = 0; i < path.size(); ++i) {
    const mav_msgs::EigenTrajectoryPoint& point = path[i];

    if (i > 0) {
      distance += (point.position_W - last_point).norm();
    }
    last_point = point.position_W;
  }

  return distance;
}

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_UTILS_H_
