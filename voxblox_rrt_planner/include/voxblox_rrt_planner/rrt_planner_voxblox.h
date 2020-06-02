#ifndef VOXBLOX_RRT_PLANNER_RRT_PLANNER_VOXBLOX_H
#define VOXBLOX_RRT_PLANNER_RRT_PLANNER_VOXBLOX_H

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_msgs/conversions.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_visualization/helpers.h>
#include <minkindr_conversions/kindr_msg.h>
#include <voxblox_ros/esdf_server.h>

#include "voxblox_rrt_planner/ompl_rrt_voxblox.h"
#include <mav_planning_voxblox/voxblox_planner.h>
#include "voxblox_rrt_planner/rrt_planner.h"

namespace mav_planning {

class VoxbloxRrtPlanner : public RrtPlanner, public VoxbloxPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  ~VoxbloxRrtPlanner() {}

  // constructor functions
  void setupPlannerAndSmootherMap();
  void initializeMap();

 protected:
  void setupRrtPlanner();
  bool planRrt(mav_msgs::EigenTrajectoryPoint& start_pose,
      mav_msgs::EigenTrajectoryPoint& goal_pose,
      mav_msgs::EigenTrajectoryPoint::Vector* waypoints);

 private:
  // Planners!
  VoxbloxOmplRrt rrt_;
};

}  // namespace mav_planning

#endif  //VOXBLOX_RRT_PLANNER_RRT_PLANNER_VOXBLOX_H
