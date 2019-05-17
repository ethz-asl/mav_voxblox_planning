#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_rrt_planner/parent_rrt_planner.h"
#include "voxblox_rrt_planner/voxblox_rrt_planner.h"

namespace mav_planning {

VoxbloxRrtPlanner::VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : ParentRrtPlanner(nh, nh_private),
      voxblox_server_(nh_, nh_private_),
      rrt_(nh_, nh_private_) {

  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  initializeMap();

  // TODO(helenol): figure out what to do with optimistic/pessimistic here.
  rrt_.setRobotRadius(constraints_.robot_radius);
  rrt_.setOptimistic(false);

  // Set up the path smoother as well.
  smoother_.setParametersFromRos(nh_private_);
  smoother_.setMinCollisionCheckResolution(voxel_size_);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size_);

  setupPlannerAndSmootherMap();

  visualizeMap();
}

void VoxbloxRrtPlanner::initializeMap() {
  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  if (!input_filepath_.empty()) {
    // Verify that the map has an ESDF layer, otherwise generate it.
    if (!voxblox_server_.loadMap(input_filepath_)) {
      ROS_ERROR("Couldn't load ESDF map!");

      // Check if the TSDF layer is non-empty...
      if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
        ROS_INFO("Generating ESDF layer from TSDF.");
        // If so, generate the ESDF layer!

        const bool full_euclidean_distance = true;
        voxblox_server_.updateEsdfBatch(full_euclidean_distance);
      } else {
        ROS_ERROR("TSDF map also empty! Check voxel size!");
      }
    }

    voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);
  }
  voxel_size_ =
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  ROS_INFO(
      "Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());
}

void VoxbloxRrtPlanner::setupPlannerAndSmootherMap() {
  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  smoother_.setMapDistanceCallback(std::bind(&VoxbloxRrtPlanner::getMapDistance,
                                             this, std::placeholders::_1));

  loco_smoother_.setMapDistanceCallback(std::bind(
      &VoxbloxRrtPlanner::getMapDistance, this, std::placeholders::_1));
}

void VoxbloxRrtPlanner::visualizeMap() {
  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }}

bool VoxbloxRrtPlanner::isMapInitialized() {
  bool esdf_initialized = esdf_map_ &&
      esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0;
  bool tsdf_initialized = tsdf_map_ &&
      tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0;

  if (!(esdf_initialized) && !(tsdf_initialized)) {
    ROS_ERROR("Both maps are empty!");
  }

  return esdf_initialized || tsdf_initialized;
}

double VoxbloxRrtPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

void VoxbloxRrtPlanner::computeMapBounds(Eigen::Vector3d* lower_bound,
                                         Eigen::Vector3d* upper_bound) const {
  if (esdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

void VoxbloxRrtPlanner::setupRrtPlanner() {
  // Inflate the bounds a bit.
  constexpr double kBoundInflationMeters = 0.5;
  // Don't in flate in z. ;)
  rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0),
                 upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0));
  rrt_.setupProblem();
}

bool VoxbloxRrtPlanner::planRrt(mav_msgs::EigenTrajectoryPoint& start_pose,
    mav_msgs::EigenTrajectoryPoint& goal_pose,
    mav_msgs::EigenTrajectoryPoint::Vector* waypoints) {
  return rrt_.getPathBetweenWaypoints(start_pose, goal_pose, waypoints);
}

}  // namespace mav_planning
