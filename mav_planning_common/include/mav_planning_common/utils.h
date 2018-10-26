#ifndef MAV_PLANNING_COMMON_UTILS_H_
#define MAV_PLANNING_COMMON_UTILS_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <random>

namespace mav_planning {

// Computes total path length of a sampled path.
inline double computePathLength(
    const mav_msgs::EigenTrajectoryPointVector& path) {
  Eigen::Vector3d last_point;
  double distance = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    const mav_msgs::EigenTrajectoryPoint& point = path[i];

    if (i > 0) {
      distance += (point.position_W - last_point).norm();
    }
    last_point = point.position_W;
  }

  return distance;
}

// Generates a double between m and n. Use srand(x) to set the seed used for
// this.
inline double randMToN(double m, double n) {
  return m + (rand() / (RAND_MAX / (n - m)));
}

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_UTILS_H_
