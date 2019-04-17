#include <limits>

#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <voxblox/utils/neighbor_tools.h>

#include "voxblox_loco_planner/shotgun_planner.h"

namespace mav_planning {

void ShotgunPlanner::setEsdfMap(
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map) {
  CHECK(esdf_map);
  esdf_map_ = esdf_map;
}

void ShotgunPlanner::setSeed(int seed) { srand(seed); }

// Main function to call. Returns whether the particles were able to get
// anywhere at all.
bool ShotgunPlanner::shootParticles(
    int num_particles, int max_steps, const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal, Eigen::Vector3d* best_goal,
    voxblox::AlignedVector<Eigen::Vector3d>* best_path) {
  mav_trajectory_generation::timing::Timer timer("loco/shotgun");

  CHECK_NOTNULL(best_goal);
  CHECK_NOTNULL(best_path);
  if (!esdf_map_) {
    return false;
  }
  voxblox::Layer<voxblox::EsdfVoxel>* layer = esdf_map_->getEsdfLayerPtr();
  CHECK_NOTNULL(layer);
  float voxel_size = layer->voxel_size();

  bool success = false;

  // Figure out the voxel index of the start point.
  voxblox::GlobalIndex start_index =
      voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
          start.cast<voxblox::FloatingPoint>(), 1.0/voxel_size);
  voxblox::GlobalIndex goal_index =
      voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
          goal.cast<voxblox::FloatingPoint>(), 1.0/voxel_size);

  voxblox::GlobalIndex current_index, last_index, best_index;
  double best_distance = std::numeric_limits<double>::max();
  voxblox::Neighborhood<>::IndexMatrix neighbors;

  for (int n_particle = 0; n_particle < num_particles; n_particle++) {
    current_index = start_index;
    for (int step = 0; step < max_steps; step++) {
      // Get the neighbors of the current index.
      voxblox::Neighborhood<>::getFromGlobalIndex(current_index, &neighbors);

      voxblox::AlignedVector<voxblox::GlobalIndex> valid_neighbors;

      // Check which neighbors are valid (traversible and observed).
      for (int i = 0; i < neighbors.cols(); i++) {
        // These are columns for some reason????? Why??????????
        const voxblox::GlobalIndex& neighbor = neighbors.col(i);
        voxblox::EsdfVoxel* esdf_voxel =
            layer->getVoxelPtrByGlobalIndex(neighbor);
        if (esdf_voxel == nullptr || !esdf_voxel->observed ||
            esdf_voxel->distance < constraints_.robot_radius) {
          continue;
        }
        // Don't go backwards at least at this step.
        if (step > 0 && neighbor == last_index) {
          continue;
        }
        valid_neighbors.push_back(neighbor);
      }

      // Nowhere for us to go. :(
      if (valid_neighbors.empty()) {
        break;
      }

      // Select one to go to.
      last_index = current_index;
      double best_goal_distance = std::numeric_limits<double>::max();
      for (const voxblox::GlobalIndex& neighbor : valid_neighbors) {
        double neighbor_goal_distance = (goal_index - neighbor).norm();
        if (neighbor_goal_distance < best_goal_distance) {
          best_goal_distance = neighbor_goal_distance;
          current_index = neighbor;
        }
        // Within a voxel! We're dealing with voxel coordinates here.
        if (neighbor_goal_distance < 1.0) {
          break;
        }
      }
      //std::cout << "Step " << step << " valid neighbors "
      //          << valid_neighbors.size() << " best_goal_distance "
       //         << best_goal_distance << std::endl;

      // TODO!!!! Add options other than just goal-seeking.
    }

    // Evaluate how good the current point is.
    double current_distance = (goal_index - current_index).norm();
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_index = current_index;
    }
  }

  *best_goal = (voxblox::getCenterPointFromGridIndex(best_index, voxel_size))
                   .cast<double>();
  return true;
}

}  // namespace mav_planning
