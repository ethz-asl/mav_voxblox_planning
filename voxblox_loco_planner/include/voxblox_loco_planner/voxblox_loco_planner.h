#ifndef VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_
#define VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_

#include <loco_planner/loco.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox/core/common.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox_planning_common/path_shortening.h>

#include "voxblox_loco_planner/shotgun_planner.h"

namespace mav_planning {

class VoxbloxLocoPlanner {
 public:
  static constexpr int kN = 10;
  static constexpr int kD = 3;

  VoxbloxLocoPlanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  // MUST be called to associate the map with the planner.
  void setEsdfMap(const std::shared_ptr<voxblox::EsdfMap>& esdf_map);

  bool getTrajectoryTowardGoal(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_trajectory_generation::Trajectory* trajectory);

  bool getTrajectoryTowardGoalFromInitialTrajectory(
      double start_time,
      const mav_trajectory_generation::Trajectory& trajectory_in,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_trajectory_generation::Trajectory* trajectory);

  bool getTrajectoryBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      const mav_msgs::EigenTrajectoryPointVector& initial_path,
      mav_trajectory_generation::Trajectory* trajectory);

 private:
  // Callbacks to bind to loco.
  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;
  double getMapDistanceAndGradientVector(const Eigen::VectorXd& position,
                                         Eigen::VectorXd* gradient) const;

  // Evaluate what we've got here.
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

  // Intermediate goal-finding.
  bool findIntermediateGoal(const mav_msgs::EigenTrajectoryPoint& start,
                            const mav_msgs::EigenTrajectoryPoint& goal,
                            double step_size,
                            mav_msgs::EigenTrajectoryPoint* goal_out) const;

  bool findIntermediateGoalShotgun(
      const mav_msgs::EigenTrajectoryPoint& start_point,
      const mav_msgs::EigenTrajectoryPoint& goal_point,
      mav_msgs::EigenTrajectoryPoint* goal_out,
      mav_msgs::EigenTrajectoryPointVector* path_out);

  bool getNearestFreeSpaceToPoint(const Eigen::Vector3d& pos, double step_size,
                                  Eigen::Vector3d* new_pos) const;

  // Set up an initial trajectory solution to start from.
  bool getInitialTrajectory(
      const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
      double total_time,
      mav_trajectory_generation::Trajectory* trajectory) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Debug publishers.
  ros::Publisher planning_marker_pub_;

  // Settings for physical constriants.
  PhysicalConstraints constraints_;

  // General settings.
  bool verbose_;
  bool visualize_;
  std::string frame_id_;

  // Loco settings.
  int num_segments_;
  int num_random_restarts_;
  double random_restart_magnitude_;
  double planning_horizon_m_;
  bool use_shotgun_;
  bool use_shotgun_path_;

  // Planner.
  loco_planner::Loco<kN> loco_;

  // Optional intermediate planner.
  ShotgunPlanner shotgun_;
  EsdfPathShortener path_shortener_;

  // Map.
  std::shared_ptr<voxblox::EsdfMap> esdf_map_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_
