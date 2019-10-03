#include <limits>

#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <voxblox/utils/neighbor_tools.h>

#include "voxblox_loco_planner/shotgun_planner.h"

namespace mav_planning {

void ShotgunPlanner::setParametersFromRos(const ros::NodeHandle& nh) {
  nh.param("robot_radius_inflation", params_.robot_radius_inflation,
           params_.robot_radius_inflation);
  nh.param("probability_follow_goal", params_.probability_follow_goal,
           params_.probability_follow_goal);
  nh.param("probability_follow_gradient", params_.probability_follow_gradient,
           params_.probability_follow_gradient);
}

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
          start.cast<voxblox::FloatingPoint>(), 1.0 / voxel_size);
  voxblox::GlobalIndex goal_index =
      voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
          goal.cast<voxblox::FloatingPoint>(), 1.0 / voxel_size);

  voxblox::GlobalIndex current_index, last_index, best_index;
  double best_distance = std::numeric_limits<double>::max();
  voxblox::Neighborhood<>::IndexMatrix neighbors;
  voxblox::AlignedVector<voxblox::GlobalIndex> best_path_voxblox;

  for (int n_particle = 0; n_particle < num_particles; n_particle++) {
    bool exit_loop = false;

    current_index = start_index;
    voxblox::AlignedVector<voxblox::GlobalIndex> current_path;
    current_path.push_back(start_index);
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

      // Select an option.
      Decision decision = selectDecision(n_particle);

      // Option 1: select the best goal distance.
      if (decision == kFollowGoal) {
        double best_goal_distance = std::numeric_limits<double>::max();
        for (const voxblox::GlobalIndex& neighbor : valid_neighbors) {
          double neighbor_goal_distance = (goal_index - neighbor).norm();
          if (neighbor_goal_distance < best_goal_distance) {
            best_goal_distance = neighbor_goal_distance;
            current_index = neighbor;
          }
        }

        // Within a voxel! We're dealing with voxel coordinates here.
        if (best_goal_distance < 1.0) {
          exit_loop = true;
          break;
        }
      } else if (decision == kFollowGradient) {
        // Then we select the neighbor that's the furthest away.
        float highest_obstacle_distance = 0.0;
        for (const voxblox::GlobalIndex& neighbor : valid_neighbors) {
          voxblox::EsdfVoxel* esdf_voxel =
              layer->getVoxelPtrByGlobalIndex(neighbor);
          float neighbor_obstacle_distance = esdf_voxel->distance;
          if (neighbor_obstacle_distance > highest_obstacle_distance) {
            highest_obstacle_distance = neighbor_obstacle_distance;
            current_index = neighbor;
          }
        }
      } else if (decision == kRandom) {
        size_t random_vec_index = static_cast<size_t>(
            std::round(randMToN(0.0, valid_neighbors.size() - 1.0)));
        current_index = valid_neighbors[random_vec_index];
      }

      const int kStepSize = 10;
      if (step % kStepSize == 0) {
        current_path.push_back(current_index);
      }
    }

    // Evaluate how good the current point is.
    double current_distance = (goal_index - current_index).norm();
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_index = current_index;
      best_path_voxblox = current_path;
      best_path_voxblox.push_back(current_index);
    }
    if (exit_loop) {
      break;
    }
  }

  if (best_index == goal_index) {
    *best_goal = goal;
  } else {
    *best_goal = (voxblox::getCenterPointFromGridIndex(best_index, voxel_size))
                     .cast<double>();
  }

  // Convert best path to voxel coordinates.
  if (best_path != nullptr) {
    best_path->reserve(best_path_voxblox.size());
    for (const voxblox::GlobalIndex& voxel_index : best_path_voxblox) {
      best_path->push_back(
          (voxblox::getCenterPointFromGridIndex(voxel_index, voxel_size))
              .cast<double>());
    }
  }
  return true;
}

ShotgunPlanner::Decision ShotgunPlanner::selectDecision(int n_particle) const {
  // ALWAYS just select goal-seeking for the first particle.
  if (n_particle == 0) {
    return kFollowGoal;
  }

  double random_probability = randMToN(0.0, 1.0);
  if (random_probability < params_.probability_follow_goal) {
    return kFollowGoal;
  } else if (random_probability < params_.probability_follow_goal +
                                      params_.probability_follow_gradient) {
    return kFollowGradient;
  } else {
    return kRandom;
  }
}

}  // namespace mav_planning
