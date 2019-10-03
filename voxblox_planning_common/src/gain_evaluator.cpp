#include <mav_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "voxblox_planning_common/gain_evaluator.h"

namespace mav_planning {

GainEvaluator::GainEvaluator() {}

void GainEvaluator::setCameraModelParametersFoV(double horizontal_fov,
                                                double vertical_fov,
                                                double min_distance,
                                                double max_distance) {
  cam_model_.setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance,
                                  max_distance);
}

void GainEvaluator::setCameraModelParametersFocalLength(
    const Eigen::Vector2d& resolution, double focal_length, double min_distance,
    double max_distance) {
  cam_model_.setIntrinsicsFromFocalLength(
      resolution.cast<float>(), focal_length, min_distance, max_distance);
}

void GainEvaluator::setCameraExtrinsics(const voxblox::Transformation& T_C_B) {
  cam_model_.setExtrinsics(T_C_B);
}

void GainEvaluator::setTsdfLayer(
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  voxel_size_ = tsdf_layer_->voxel_size();
  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

double GainEvaluator::evaluateExplorationGainVoxelCount(
    const mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  mav_trajectory_generation::timing::Timer timer_gain("exploration/exp_gain");

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the boundaries of the current view.
  Eigen::Vector3f aabb_min, aabb_max;
  cam_model_.getAabb(&aabb_min, &aabb_max);

  double num_unknown = 0.0;

  // Since some complete blocks may be unallocated, just do the dumbest possible
  // thing: iterate over all voxels in the AABB and check if they belong (which
  // should be quite cheap), then look them up.
  double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;
  Eigen::Vector3f pos = aabb_min;
  for (pos.x() = aabb_min.x(); pos.x() < aabb_max.x(); pos.x() += voxel_size_) {
    for (pos.y() = aabb_min.y(); pos.y() < aabb_max.y();
         pos.y() += voxel_size_) {
      for (pos.z() = aabb_min.z(); pos.z() < aabb_max.z();
           pos.z() += voxel_size_) {
        if (!cam_model_.isPointInView(pos)) {
          continue;
        }
        if (voxel_index % modulus != 0) {
          voxel_index++;
          continue;
        }
        voxel_index++;
        checked_voxels++;

        // Look up the voxel in the TSDF.
        voxblox::BlockIndex block_index =
            tsdf_layer_->computeBlockIndexFromCoordinates(pos);

        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index);
        if (block_ptr) {
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByCoordinates(pos);
          if (voxel.weight <= 1e-1) {
            num_unknown++;
          }
        } else {
          num_unknown++;
        }
      }
    }
  }
  // Divide percentages by the checked voxels.
  num_unknown /= checked_voxels;

  timer_gain.Stop();
  return num_unknown;
}

double GainEvaluator::evaluateExplorationGainWithRaycasting(
    const mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  mav_trajectory_generation::timing::Timer timer_gain(
      "exploration/exp_gain_raycast");

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double num_unknown = 0.0;
  double num_occluded = 0.0;
  double num_free = 0.0;
  double num_occupied = 0.0;

  // This is a set of all the checked voxels so that they don't get checked
  // multiple times...
  voxblox::HierarchicalIndexSet checked_voxels_set;

  // Get the plane bounds for the back plane of the frustum and just iterate
  // over these points.
  // For each voxel, cast a ray to the camera center and check whether it is
  // occluded or not.
  double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;

  // Get the three points defining the back plane of the camera frustum.
  voxblox::AlignedVector<voxblox::Point> plane_points;
  cam_model_.getFarPlanePoints(&plane_points);

  // We map the plane into u and v coordinates, which are the plane's coordinate
  // system, with the origin at plane_points[1] and outer bounds at
  // plane_points[0] and plane_points[2].
  Eigen::Vector3f u_distance = plane_points[0] - plane_points[1];
  Eigen::Vector3f u_slope = u_distance.normalized();
  int u_max = static_cast<int>(
      std::ceil(u_distance.norm() * voxel_size_inv_));  // Round this up.

  Eigen::Vector3f v_distance = plane_points[2] - plane_points[1];
  Eigen::Vector3f v_slope = v_distance.normalized();
  int v_max = static_cast<int>(
      std::ceil(v_distance.norm() * voxel_size_inv_));  // Round this up.

  // We then iterate over all the voxels in the coordinate space of the back
  // bounding plane of the frustum.
  Eigen::Vector3f pos = plane_points[1];
  for (int u = 0; u < u_max; u++) {
    for (int v = 0; v < v_max; v++) {
      if (voxel_index % modulus != 0) {
        voxel_index++;
        continue;
      }
      voxel_index++;

      // Get the 'real' coordinates back from the plane coordinate space.
      pos = plane_points[1] + u * u_slope * voxel_size_ +
            v * v_slope * voxel_size_;

      // Get the block + voxel index of this voxel by projecting it into
      // the voxel grid and then computing from the global index.
      // This is a truncating cast, which is I think what we want in this
      // case.
      voxblox::GlobalIndex global_voxel_idx =
          (voxel_size_inv_ * pos).cast<voxblox::LongIndexElement>();
      voxblox::BlockIndex block_index =
          voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                     voxels_per_side_inv_);
      voxblox::VoxelIndex voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
          global_voxel_idx, voxels_per_side_);

      // Should we check if we already cast this?
      if (checked_voxels_set[block_index].count(voxel_index) > 0) {
        continue;
      }
      // Otherwise we should probably cast the ray through it and then
      // look up all the voxels and count them.
      const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
      const voxblox::Point end_scaled = pos * voxel_size_inv_;

      voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
      voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

      // Iterate over all the voxels in the index in order.
      // Put them in the checked queue, and classify them. We're starting from
      // the camera center to the current pose, so this defines how we handle
      // occlusions.
      int unknown_ray = 0;
      int occluded_ray = 0;
      int free_ray = 0;
      int occupied_ray = 0;
      bool ray_occluded = false;
      for (int i = 0; i < global_voxel_indices.size(); i++) {
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];
        voxblox::BlockIndex block_index_ray =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                       voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index_ray =
            voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                  voxels_per_side_);

        bool voxel_checked = false;
        // Check if this is already checked; we don't add it to the counts
        // in that case.
        if (checked_voxels_set[block_index_ray].count(voxel_index_ray) > 0) {
          voxel_checked = true;
        }
        voxblox::Point recovered_pos =
            global_voxel_idx.cast<float>() * voxel_size_;
        if (!voxel_checked && !cam_model_.isPointInView(recovered_pos)) {
          // If not in frustum, don't count contributions from this point.
          voxel_checked = true;
        }

        // This is as far as we need to go if this ray is already occluded.
        if (ray_occluded) {
          if (!voxel_checked) {
            occluded_ray++;
            checked_voxels++;
            checked_voxels_set[block_index_ray].insert(voxel_index_ray);
          }
          continue;
        }

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight <= 1e-1) {
            if (!voxel_checked) {
              unknown_ray++;
            }
          } else if (voxel.distance <= 0.0) {
            // This is an occupied voxel! Mark all the stuff behind
            // it as occluded.
            if (!voxel_checked) {
              occupied_ray++;
            }
            ray_occluded = true;
          } else {
            if (!voxel_checked) {
              free_ray++;
            }
          }
        } else {
          if (!voxel_checked) {
            unknown_ray++;
          }
        }

        if (!voxel_checked) {
          checked_voxels++;
          checked_voxels_set[block_index_ray].insert(voxel_index_ray);
        }
      }

      // Now that we have the whole ray, add up the counts.
      num_unknown += unknown_ray;
      num_free += free_ray;
      num_occluded += occluded_ray;
      num_occupied += occupied_ray;
    }
  }
  // Divide percentages by the checked voxels.
  num_unknown /= checked_voxels;

  timer_gain.Stop();
  return num_unknown;
}

double GainEvaluator::evaluateExplorationGainBircher(
    const mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  mav_trajectory_generation::timing::Timer timer_gain(
      "exploration/exp_gain_bircher");

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the boundaries of the current view.
  Eigen::Vector3f aabb_min, aabb_max;
  cam_model_.getAabb(&aabb_min, &aabb_max);

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double num_unknown = 0.0;
  double num_occluded = 0.0;
  double num_free = 0.0;
  double num_occupied = 0.0;

  // Since some complete blocks may be unallocated, just do the dumbest possible
  // thing: iterate over all voxels in the AABB and check if they belong (which
  // should be quite cheap), then look them up.
  double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;
  Eigen::Vector3f pos = aabb_min.cast<float>();
  for (pos.x() = aabb_min.x(); pos.x() < aabb_max.x(); pos.x() += voxel_size_) {
    for (pos.y() = aabb_min.y(); pos.y() < aabb_max.y();
         pos.y() += voxel_size_) {
      for (pos.z() = aabb_min.z(); pos.z() < aabb_max.z();
           pos.z() += voxel_size_) {
        if (!cam_model_.isPointInView(pos)) {
          continue;
        }
        if (voxel_index % modulus != 0) {
          voxel_index++;
          continue;
        }
        voxel_index++;
        checked_voxels++;

        // Get the block + voxel index of this voxel by projecting it into
        // the voxel grid and then computing from the global index.
        // This is a truncating cast, which is I think what we want in this
        // case.
        voxblox::GlobalIndex global_voxel_idx =
            (voxel_size_inv_ * pos).cast<voxblox::LongIndexElement>();
        voxblox::BlockIndex block_index =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                       voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_);

        // Check if this voxel is occluded.
        const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
        const voxblox::Point end_scaled = pos * voxel_size_inv_;

        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        // Iterate over all the voxels in the index in order.
        // Put them in the checked queue, and classify them. We're starting from
        // the camera center to the current pose, so this defines how we handle
        // occlusions. Don't raycast the last voxel, since it's the actual
        // voxel we're checking (ok if it's occupied, still not an occlusion).
        bool ray_occluded = false;
        for (int i = 0; i < global_voxel_indices.size() - 1; i++) {
          const voxblox::GlobalIndex& global_voxel_idx =
              global_voxel_indices[i];
          voxblox::BlockIndex block_index_ray =
              voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                         voxels_per_side_inv_);
          voxblox::VoxelIndex voxel_index_ray =
              voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                    voxels_per_side_);

          // Otherwise look up this voxel and add it to checked.
          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(block_index_ray);
          if (block_ptr) {
            // If this block exists, get the voxel.
            const voxblox::TsdfVoxel& voxel =
                block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
            if (voxel.weight > 1e-1 && voxel.distance <= 0.0) {
              // This is an occupied voxel! Mark all the stuff behind
              // it as occluded.
              ray_occluded = true;
              break;
            }
          }
        }
        if (ray_occluded) {
          num_occluded++;
        } else {
          // If it's not occluded, ACTUALLY look up this voxel.
          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(block_index);
          if (block_ptr) {
            const voxblox::TsdfVoxel& voxel =
                block_ptr->getVoxelByCoordinates(pos);
            if (voxel.weight <= 1e-1) {
              num_unknown++;
            } else if (voxel.distance >= 0.0) {
              num_free++;
            } else {
              num_occupied++;
            }
          } else {
            num_unknown++;
          }
        }
      }
    }
  }
  timer_gain.Stop();

  // Divide percentages by the checked voxels.
  num_unknown /= checked_voxels;

  return num_unknown;
}

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}

}  // namespace mav_planning
