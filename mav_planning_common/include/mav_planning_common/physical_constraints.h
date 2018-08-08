#ifndef MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_
#define MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_

#include <ros/ros.h>
#include <math.h>

namespace mav_planning {

// Physical constraints for all planners.
struct PhysicalConstraints {
 public:
  PhysicalConstraints()
      : v_max(1.0),
        a_max(2.0),
        yaw_rate_max(M_PI / 4.0),
        robot_radius(1.0),
        sampling_dt(0.01) {}

  void setParametersFromRos(const ros::NodeHandle& nh) {
    nh.param("v_max", v_max, v_max);
    nh.param("a_max", a_max, a_max);
    nh.param("yaw_rate_max", yaw_rate_max, yaw_rate_max);
    nh.param("robot_radius", robot_radius, robot_radius);
    nh.param("sampling_dt", sampling_dt, sampling_dt);
  }

  double v_max;  // Meters/second
  double a_max;  // Meters/second^2
  double yaw_rate_max;  // Rad/second
  double robot_radius;  // Meters
  double sampling_dt;  // Seconds
};

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_
