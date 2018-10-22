#include "voxblox_loco_planner/voxblox_loco_planner.h"

namespace mav_planning {

VoxbloxLocoPlanner::VoxbloxLocoPlanner(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      visualize_(true),
      frame_id_("odom"),
      loco_(3) {
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  loco_.setRobotRadius(constraints_.robot_radius);
}

void VoxbloxLocoPlanner::setEsdfMap(
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map) {
  CHECK(esdf_map);
  esdf_map_ = esdf_map;

  loco_.setDistanceAndGradientFunction(
      std::bind(&VoxbloxLocoPlanner::getMapDistanceAndGradientVector, this,
                std::placeholders::_1, std::placeholders::_2));
  loco_.setMapResolution(esdf_map->voxel_size());
}

double VoxbloxLocoPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAtPosition(position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double VoxbloxLocoPlanner::getMapDistanceAndGradientVector(
    const Eigen::VectorXd& position, Eigen::VectorXd* gradient) const {
  CHECK_EQ(position.size(), 3);
  if (gradient == nullptr) {
    return getMapDistanceAndGradient(position, nullptr);
  }
  Eigen::Vector3d gradient_3d;
  double distance = getMapDistanceAndGradient(position, &gradient_3d);
  *gradient = gradient_3d;
  return distance;
}

double VoxbloxLocoPlanner::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                   &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

// Evaluate what we've got here.
bool VoxbloxLocoPlanner::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      return false;
    }
  }
  return true;
}

bool VoxbloxLocoPlanner::isPathFeasible(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max + 1e-2) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max + 1e-2) {
      return false;
    }
  }
  return true;
}

}  // namespace mav_planning
