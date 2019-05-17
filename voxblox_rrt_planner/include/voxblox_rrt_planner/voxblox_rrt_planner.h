#ifndef VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H
#define VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H

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

#include "voxblox_rrt_planner/voxblox_ompl_rrt.h"
#include "voxblox_rrt_planner/parent_rrt_planner.h"

namespace mav_planning {

class VoxbloxRrtPlanner : public ParentRrtPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  ~VoxbloxRrtPlanner() {}

  // constructor functions
  void initializeMap();
  void setupPlannerAndSmootherMap();
  void visualizeMap();

  // map functions
  bool isMapInitialized();
  double getMapDistance(const Eigen::Vector3d& position) const;

 protected:
  void computeMapBounds(Eigen::Vector3d* lower_bound,
                        Eigen::Vector3d* upper_bound) const;
  void setupRrtPlanner();
  bool planRrt(mav_msgs::EigenTrajectoryPoint& start_pose,
      mav_msgs::EigenTrajectoryPoint& goal_pose,
      mav_msgs::EigenTrajectoryPoint::Vector* waypoints);

 private:
  // Planners!
  VoxbloxOmplRrt rrt_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H
