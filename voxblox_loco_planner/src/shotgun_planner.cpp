#include <mav_planning_common/utils.h>

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
  CHECK_NOTNULL(best_goal);
  CHECK_NOTNULL(best_path);
  if (!esdf_map_) {
    return false;
  }

  bool success = false;
  for (int n_particle = 0; n_particle < num_particles; n_particle++) {
    for (int step = 0; step < max_steps; step++) {
    }
  }
  return true;
}

}  // namespace mav_planning
