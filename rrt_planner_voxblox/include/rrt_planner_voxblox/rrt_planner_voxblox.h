#ifndef RRT_PLANNER_VOXBLOX_RRT_PLANNER_VOXBLOX_H
#define RRT_PLANNER_VOXBLOX_RRT_PLANNER_VOXBLOX_H

#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>

#include <mav_msgs/conversions.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_visualization/helpers.h>
#include <minkindr_conversions/kindr_msg.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>

#include "rrt_planner_voxblox/ompl_rrt_voxblox.h"

namespace mav_planning {

class RrtPlannerVoxblox {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RrtPlannerVoxblox(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~RrtPlannerVoxblox() {}

  bool plannerServiceCallback(
      mav_planning_msgs::PlannerServiceRequest& request,
      mav_planning_msgs::PlannerServiceResponse& response);

  bool publishPathCallback(
      std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  // Tools for trajectory smoothing and checking.
  bool generateFeasibleTrajectory(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path,
      bool* path_is_in_collision_with_gt = nullptr);
  bool generateFeasibleTrajectoryLoco(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path,
      bool* path_is_in_collision_with_gt = nullptr);
  bool generateFeasibleTrajectoryLoco2(
      const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path,
      bool* path_is_in_collision_with_gt = nullptr);

  double getMapDistance(const Eigen::Vector3d& position) const;
  bool checkPathForCollisions(
      const mav_msgs::EigenTrajectoryPointVector& path, double* t) const;

  bool checkPathForCollisionsWithGroundTruth(
      const mav_msgs::EigenTrajectoryPointVector& path, double* t) const;
  double getMapDistanceGroundTruth(const Eigen::Vector3d& position) const;

  bool checkPhysicalConstraints(
      const mav_trajectory_generation::Trajectory& trajectory);

  // Visualization utilities.
  visualization_msgs::Marker createMarkerForPath(
      mav_msgs::EigenTrajectoryPointVector& path,
      const std_msgs::ColorRGBA& color, const std::string& name,
      double scale = 0.05) const;
  visualization_msgs::Marker createMarkerForWaypoints(
      mav_msgs::EigenTrajectoryPointVector& path,
      const std_msgs::ColorRGBA& color, const std::string& name,
      double scale = 0.05) const;

  visualization_msgs::Marker createPlanningStatusMarker(
      const size_t plan_number, const bool rrt_success, const bool loco_success,
      const bool loco_gt_violation, const bool loco_2_success,
      const bool loco_2_gt_violation, const bool poly_success,
      const bool poly_gt_violation) const;

 private:
  void inferValidityCheckingResolution(const Eigen::Vector3d& bounding_box);

  bool sanityCheckWorldAndInputs(
      const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos,
      const Eigen::Vector3d& bounding_box) const;
  bool checkStartAndGoalFree(
      const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos) const;

  void computeMapBounds(
      Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher polynomial_trajectory_pub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_list_pub_;
  ros::Publisher status_pub_;

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

  voxblox::EsdfMap::Ptr esdf_ground_truth_map_;

  double robot_radius_margin_ = 0.2;
  int num_repetitions_ = 10;

  // Planners!
  OmplRrtVoxblox rrt_;

  // Smoothing!
  PolynomialSmoother smoother_;
  LocoSmoother loco_smoother_;

  bool enable_loco_;
  bool enable_loco_2_;
  bool enable_poly_;

  float marker_font_size_;
  float status_position_x_;
  float status_position_y_;
  float status_position_z_;
};  // namespace mav_planning

struct PlanningResult {
  size_t paths_planned = 0u;

  size_t poly_self_collisions = 0u;
  size_t poly_gt_collisions = 0u;
  size_t poly_success = 0u;

  double average_successful_poly_path_length_m = 0.0;

  size_t loco_self_collisions = 0u;
  size_t loco_gt_collisions = 0u;
  size_t loco_success = 0u;

  double average_successful_loco_path_length_m = 0.0;

  size_t loco_2_self_collisions = 0u;
  size_t loco_2_gt_collisions = 0u;
  size_t loco_2_success = 0u;

  double average_successful_loco_2_path_length_m = 0.0;

  std::string print() const {
    std::stringstream ss;
    ss << "\n\n PLANNING RESULTS \n"
       << "\npaths planned:          " << paths_planned
       << "\npoly_self_collisions:   " << poly_self_collisions << "\t"
       << (static_cast<double>(poly_self_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\npoly_gt_collisions:     " << poly_gt_collisions << "\t"
       << (static_cast<double>(poly_gt_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\npoly_success:           " << poly_success << "\t"
       << (static_cast<double>(poly_success) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\npoly_avg_path_length_m: " << average_successful_poly_path_length_m
       << "m"
       << "\n\nloco_self_collisions:   " << loco_self_collisions << "\t"
       << (static_cast<double>(loco_self_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_gt_collisions:     " << loco_gt_collisions << "\t"
       << (static_cast<double>(loco_gt_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_success:           " << loco_success << "\t"
       << (static_cast<double>(loco_success) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_avg_path_length_m: " << average_successful_loco_path_length_m
       << "m"
       << "\n\nloco_2_self_collisions:   " << loco_2_self_collisions << "\t"
       << (static_cast<double>(loco_2_self_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_2_gt_collisions:     " << loco_2_gt_collisions << "\t"
       << (static_cast<double>(loco_2_gt_collisions) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_2_success:           " << loco_2_success << "\t"
       << (static_cast<double>(loco_2_success) /
           static_cast<double>(paths_planned) * 100.0)
       << "%"
       << "\nloco_2_avg_path_length_m: "
       << average_successful_loco_2_path_length_m << "m";
    return ss.str();
  }
};

}  // namespace mav_planning

#endif  // RRT_PLANNER_VOXBLOX_RRT_PLANNER_VOXBLOX_H
