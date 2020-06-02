#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_rrt_planner/rrt_planner.h"
#include "voxblox_rrt_planner/rrt_planner_voxblox.h"

namespace mav_planning {

VoxbloxRrtPlanner::VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : RrtPlanner(nh, nh_private),
      VoxbloxPlanner(nh_, nh_private_),
      rrt_(nh_, nh_private_) {

  map_ = this;

  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  initializeMap();

  // TODO(helenol): figure out what to do with optimistic/pessimistic here.
  rrt_.setRobotRadius(constraints_.robot_radius);
  rrt_.setOptimistic(false);
  rrt_.setPlanner(VoxbloxOmplRrt::kRrtConnect);

  // Set up the path smoother as well.
  smoother_.setParametersFromRos(nh_private_);
  smoother_.setMinCollisionCheckResolution(map_->getVoxelSize());

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(map_->getVoxelSize());

  setupPlannerAndSmootherMap();

  visualizeMap();
}

void VoxbloxRrtPlanner::initializeMap() {
  VoxbloxPlanner::initializeMap();

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);
}

void VoxbloxRrtPlanner::setupPlannerAndSmootherMap() {
  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  smoother_.setMapDistanceCallback(std::bind(&VoxbloxRrtPlanner::getMapDistance,
                                             this, std::placeholders::_1));

  loco_smoother_.setMapDistanceCallback(std::bind(
      &VoxbloxRrtPlanner::getMapDistance, this, std::placeholders::_1));
}

void VoxbloxRrtPlanner::setupRrtPlanner() {
  Eigen::Vector3d lower_bound, upper_bound;
  map_->computeMapBounds(&lower_bound, &upper_bound);

  // Inflate the bounds a bit.
  constexpr double kBoundInflationMeters = 0.5;
  // Don't in flate in z. ;)
  rrt_.setBounds(lower_bound - Eigen::Vector3d(kBoundInflationMeters,
                                               kBoundInflationMeters, 0.0),
                 upper_bound + Eigen::Vector3d(kBoundInflationMeters,
                                               kBoundInflationMeters, 0.0));
  rrt_.setupProblem();
}

bool VoxbloxRrtPlanner::planRrt(mav_msgs::EigenTrajectoryPoint& start_pose,
    mav_msgs::EigenTrajectoryPoint& goal_pose,
    mav_msgs::EigenTrajectoryPoint::Vector* waypoints) {
  return rrt_.getPathBetweenWaypoints(start_pose, goal_pose, waypoints);
}

}  // namespace mav_planning
