#include "voxblox_rrt_planner/ompl_rrt_voxblox.h"

namespace mav_planning {

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : BloxOmplRrt(nh, nh_private) {}

void VoxbloxOmplRrt::setTsdfLayer(
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  CHECK_NOTNULL(tsdf_layer_);
  voxel_size_ = tsdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setEsdfLayer(
    voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
  esdf_layer_ = esdf_layer;
  CHECK_NOTNULL(esdf_layer_);
  voxel_size_ = esdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setupProblem(){
  problem_setup_ = &problem_setup_voxblox_;
  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    problem_setup_voxblox_.setTsdfVoxbloxCollisionChecking(robot_radius_, tsdf_layer_);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_voxblox_.setEsdfVoxbloxCollisionChecking(robot_radius_, esdf_layer_);
  }

  BloxOmplRrt::setupProblem();
}
}  // namespace mav_planning
