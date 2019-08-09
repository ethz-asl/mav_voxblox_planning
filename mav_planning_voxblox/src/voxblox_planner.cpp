#include <mav_planning_voxblox/voxblox_planner.h>

#include <voxblox/utils/planning_utils.h>

namespace mav_planning {

VoxbloxPlanner::VoxbloxPlanner(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : voxblox_server_(nh, nh_private),
      visualize_(false) {
//  nh_private.param("visualize", visualize_, visualize_);
  nh_private.param("voxblox_path", input_filepath_, input_filepath_);
}

void VoxbloxPlanner::initializeMap() {
  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  if (!input_filepath_.empty()) {
    // Verify that the map has an ESDF layer, otherwise generate it.
    if (!voxblox_server_.loadMap(input_filepath_)) {
      ROS_ERROR("Couldn't load ESDF map!");
    }
  }

  // generate ESDF layer
  if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
    ROS_INFO("Generating ESDF layer from TSDF.");
    // If so, generate the ESDF layer!

    const bool full_euclidean_distance = true;
    voxblox_server_.updateEsdfBatch(full_euclidean_distance);
  } else {
    ROS_ERROR("TSDF map also empty! Check voxel size!");
  }

//  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  voxel_size_ =
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  ROS_INFO(
      "[VoxbloxPlanner] Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());
}

void VoxbloxPlanner::visualizeMap() {
  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }
}

bool VoxbloxPlanner::isMapInitialized() {
  bool esdf_initialized = esdf_map_ &&
                          esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0;
  bool tsdf_initialized = tsdf_map_ &&
                          tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0;

  if (!(esdf_initialized) && !(tsdf_initialized)) {
    ROS_ERROR("Both maps are empty!");
  }

  return esdf_initialized || tsdf_initialized;
}

double VoxbloxPlanner::getMapDistance(const Eigen::Vector3d &position) const {
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

void VoxbloxPlanner::computeMapBounds(Eigen::Vector3d *lower_bound,
                                      Eigen::Vector3d *upper_bound) const {
  if (esdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

}