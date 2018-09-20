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

namespace mav_planning {

class VoxbloxRrtPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  virtual ~VoxbloxRrtPlanner() {}

  bool plannerServiceCallback(
      mav_planning_msgs::PlannerServiceRequest& request,
      mav_planning_msgs::PlannerServiceResponse& response);

  bool publishPathCallback(std_srvs::EmptyRequest& request,
                           std_srvs::EmptyResponse& response);

  // Tools for trajectory smoothing and checking.
  bool generateFeasibleTrajectory(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path);
  bool generateFeasibleTrajectoryLoco(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path);
  bool generateFeasibleTrajectoryLoco2(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path);

  double getMapDistance(const Eigen::Vector3d& position) const;
  bool checkPathForCollisions(const mav_msgs::EigenTrajectoryPointVector& path,
                              double* t) const;
  bool checkPhysicalConstraints(
      const mav_trajectory_generation::Trajectory& trajectory);

 private:
  void inferValidityCheckingResolution(const Eigen::Vector3d& bounding_box);

  bool sanityCheckWorldAndInputs(const Eigen::Vector3d& start_pos,
                                 const Eigen::Vector3d& goal_pos,
                                 const Eigen::Vector3d& bounding_box) const;
  bool checkStartAndGoalFree(const Eigen::Vector3d& start_pos,
                             const Eigen::Vector3d& goal_pos) const;

  void computeMapBounds(Eigen::Vector3d* lower_bound,
                        Eigen::Vector3d* upper_bound) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher polynomial_trajectory_pub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_list_pub_;

  ros::ServiceServer planner_srv_;
  ros::ServiceServer path_pub_srv_;

  std::string frame_id_;
  bool visualize_;
  bool do_smoothing_;

  // Robot parameters -- v max, a max, radius, etc.
  PhysicalConstraints constraints_;

  // Number of meters to inflate the bounding box by for straight-line planning.
  double bounding_box_inflation_m_;
  double num_seconds_to_plan_;
  bool simplify_solution_;

  // Bounds on the size of the map.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;
  double voxel_size_;  // Cache the size of the voxels used by the map.

  // Cache the last trajectory for output :)
  mav_trajectory_generation::Trajectory last_trajectory_;
  bool last_trajectory_valid_;
  mav_msgs::EigenTrajectoryPointVector last_waypoints_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;

  // Planners!
  VoxbloxOmplRrt rrt_;

  // Smoothing!
  PolynomialSmoother smoother_;
  LocoSmoother loco_smoother_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_VOXBLOX_RRT_PLANNER_H
