#ifndef VOXBLOX_RRT_PLANNER_OMPL_RRT_CBLOX_H_
#define VOXBLOX_RRT_PLANNER_OMPL_RRT_CBLOX_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include <voxblox_rrt_planner/ompl_rrt.h>
#include <voxblox_rrt_planner/ompl/mav_setup.h>

namespace mav_planning {

class CbloxOmplRrt : public BloxOmplRrt {
 public:
  CbloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : BloxOmplRrt(nh, nh_private),
        truncation_distance_(2.0) {};
  virtual ~CbloxOmplRrt() {}

  // Only call this once, only call this after setting all settings correctly.
  void setupProblem(){
    problem_setup_.setCbloxCollisionChecking(robot_radius_, voxel_size_,
        truncation_distance_, map_distance_function_);
    BloxOmplRrt::setupProblem();
  }
  void setMapDistanceCallback(
      std::function<double(const Eigen::Vector3d& position)> function) {
    map_distance_function_ = function;
  }
  void setVoxelSize(const voxblox::FloatingPoint& voxel_size) {
    voxel_size_ = voxel_size;
  }
  void setTruncationDistance(const voxblox::FloatingPoint& truncation_distance) {
    truncation_distance_ = truncation_distance;
  }

 private:
  // map of some sorts
  std::function<double(const Eigen::Vector3d& position)> map_distance_function_;
  voxblox::FloatingPoint truncation_distance_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_OMPL_RRT_CBLOX_H_
