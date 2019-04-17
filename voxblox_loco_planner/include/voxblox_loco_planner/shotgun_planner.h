#ifndef VOXBLOX_LOCO_PLANNER_SHOTGUN_PLANNER_H_
#define VOXBLOX_LOCO_PLANNER_SHOTGUN_PLANNER_H_

#include <mav_planning_common/physical_constraints.h>
#include <voxblox/core/esdf_map.h>

namespace mav_planning {

struct ShotgunParameters {
  // How large a step, in meters, to make.
  float max_step_size = 1.0;
  // Whether to take 1 voxel (false) or multi-voxel (true) steps.
  bool take_large_steps = false;

  // Probabilities (must sum up to < 1 together) that the particle will do one
  // of the following things. Rest of the probability is just random motion.
  float probability_follow_gradient = 0.25;
  float probability_follow_goal = 0.5;

  // Some params for evaluating the goodness of the point?
};

// A "planner" that uses a set of probabilistic particles to find an initial
// path toward a goal, as far as it can get.
// Uses a voxblox ESDF as the backend.
class ShotgunPlanner {
 public:
  ShotgunPlanner() {}
  ShotgunPlanner(const ShotgunParameters& params) : params_(params) {}

  const ShotgunParameters& getParams() const { return params_; }
  void setParams(const ShotgunParameters& params) { params_ = params; }

  const PhysicalConstraints& getPhysicalConstraints() const {
    return constraints_;
  }
  void setPhysicalConstraints(const PhysicalConstraints& constraints) {
    constraints_ = constraints;
    constraints_.robot_radius += 0.2;
  }

  // MUST be called to associate the map with the planner.
  void setEsdfMap(const std::shared_ptr<voxblox::EsdfMap>& esdf_map);
  void setSeed(int seed);

  // Main function to call. Returns whether the particles were able to get
  // anywhere at all.
  bool shootParticles(int num_particles, int max_steps,
                      const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                      Eigen::Vector3d* best_goal,
                      voxblox::AlignedVector<Eigen::Vector3d>* best_path);

  bool shootParticles(int num_particles, int max_steps,
                      const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                      Eigen::Vector3d* best_goal) {
    voxblox::AlignedVector<Eigen::Vector3d> best_path;
    return shootParticles(num_particles, max_steps, start, goal, best_goal,
                          &best_path);
  }

 private:
  // Settings for physical constriants.
  PhysicalConstraints constraints_;

  ShotgunParameters params_;

  // Map.
  std::shared_ptr<voxblox::EsdfMap> esdf_map_;

  // State.
};

}  // namespace mav_planning

#endif  // VOXBLOX_LOCO_PLANNER_SHOTGUN_PLANNER_H_
