#ifndef VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_
#define VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_rrt_planner/ompl/ompl_types.h"

namespace ompl {
namespace mav {

// voxblox version
template <typename VoxelType>
class VoxbloxValidityChecker : public base::StateValidityChecker {
 public:
  VoxbloxValidityChecker(const base::SpaceInformationPtr& space_info,
                         double robot_radius, voxblox::Layer<VoxelType>* layer)
      : base::StateValidityChecker(space_info),
        layer_(layer),
        robot_radius_(robot_radius) {
    CHECK_NOTNULL(layer);
    voxel_size_ = layer->voxel_size();
  }

  virtual bool isValid(const base::State* state) const {
    Eigen::Vector3d robot_position = omplToEigen(state);
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    bool collision = checkCollisionWithRobot(robot_position);
    // We check the VALIDITY of the state, and the function above returns
    // whether the state was in COLLISION.
    return !collision;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool checkCollisionWithRobot(
      const Eigen::Vector3d& robot_position) const = 0;

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex& global_index) const {
    return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
  }

  virtual double remainingDistanceToCollision(
      const Eigen::Vector3d& position) const = 0;

  float voxel_size() const { return voxel_size_; }

 protected:
  voxblox::Layer<VoxelType>* layer_;

  float voxel_size_;
  double robot_radius_;
};

class TsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::TsdfVoxel> {
 public:
  TsdfVoxbloxValidityChecker(const base::SpaceInformationPtr& space_info,
                             double robot_radius,
                             voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, tsdf_layer),
        treat_unknown_as_occupied_(false) {}

  bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
  void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
    treat_unknown_as_occupied_ = treat_unknown_as_occupied;
  }

  virtual bool checkCollisionWithRobot(
      const Eigen::Vector3d& robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                         &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
         block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex& voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        const voxblox::TsdfVoxel& tsdf_voxel =
            block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.weight < voxblox::kEpsilon) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        if (tsdf_voxel.distance <= 0.0f) {
          return true;
        }
      }
    }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
    return false;
  }

  virtual double remainingDistanceToCollision(
      const Eigen::Vector3d& position) const {
    double remaining_distance = 0;
    bool initialized = false;
    voxblox::Point robot_point = position.cast<voxblox::FloatingPoint>();

    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                         &block_voxel_list);

    for (const auto& kv :
        block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex& voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          if (treat_unknown_as_occupied_) {
            return 0.0;
          }
          continue;
        }
        const voxblox::TsdfVoxel& tsdf_voxel =
            block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.weight < voxblox::kEpsilon) {
          if (treat_unknown_as_occupied_) {
            return 0.0;
          }
          continue;
        }
        if (tsdf_voxel.distance <= 0.0f) {
          return 0.0;
        }

        if (!initialized) {
          remaining_distance = tsdf_voxel.distance
              + static_cast<double>((robot_point -
              block_ptr->computeCoordinatesFromVoxelIndex(voxel_index)).norm());
        }
        remaining_distance =
            std::min(remaining_distance, tsdf_voxel.distance
            + static_cast<double>((robot_point -
            block_ptr->computeCoordinatesFromVoxelIndex(voxel_index)).norm()));
      }
    }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
    return remaining_distance - robot_radius_;
  }

 protected:
  bool treat_unknown_as_occupied_;
};

class EsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::EsdfVoxel> {
 public:
  EsdfVoxbloxValidityChecker(const base::SpaceInformationPtr& space_info,
                             double robot_radius,
                             voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, esdf_layer),
        interpolator_(esdf_layer) {}

  virtual bool checkCollisionWithRobot(
      const Eigen::Vector3d& robot_position) const {
    constexpr bool interpolate = false;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(
        robot_position.cast<voxblox::FloatingPoint>(), &distance, interpolate);
    if (!success) {
      return true;
    }

    return robot_radius_ >= distance;
  }

  virtual double remainingDistanceToCollision(
      const Eigen::Vector3d& position) const {
    voxblox::Point robot_point = position.cast<voxblox::FloatingPoint>();
    constexpr bool interpolate = false;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(robot_point, &distance, interpolate);
    if (!success) {
      return 0.0;
    }
    return distance - robot_radius_;
  }

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex& global_index) const {
    voxblox::EsdfVoxel* voxel = layer_->getVoxelPtrByGlobalIndex(global_index);

    if (voxel == nullptr) {
      return true;
    }
    return robot_radius_ >= voxel->distance;
  }

 protected:
  // Interpolator for the layer.
  voxblox::Interpolator<voxblox::EsdfVoxel> interpolator_;
};

// Motion validator that uses either of the validity checkers above to
// validate motions at voxel resolution.
template <typename VoxelType>
class VoxbloxMotionValidator : public base::MotionValidator {
 public:
  VoxbloxMotionValidator(
      const base::SpaceInformationPtr& space_info,
      typename std::shared_ptr<VoxbloxValidityChecker<VoxelType> >
          validity_checker)
      : base::MotionValidator(space_info), validity_checker_(validity_checker) {
    CHECK(validity_checker);
  }

  virtual bool checkMotion(const base::State* s1, const base::State* s2) const {
    std::pair<base::State*, double> unused;
    return checkMotion(s1, s2, unused);
  }

  virtual bool checkMotionSampling(const base::State* s1, const base::State* s2,
      std::pair<base::State*, double>& last_valid) const {
    Eigen::Vector3d start = omplToEigen(s1);
    Eigen::Vector3d goal = omplToEigen(s2);

    // cast ray from start to finish
    Eigen::Vector3d ray_direction = (goal - start).normalized();
    double ray_length = (goal-start).norm();
    double step_size_temp = 0;

    // iterate along ray
    Eigen::Vector3d position = start;
    bool collision;
    while ((position - start).norm() < ray_length) {
      // check for collision
      collision = validity_checker_->checkCollisionWithRobot(position);
      double remaining_distance =
          validity_checker_->remainingDistanceToCollision(position);

      // find last valid and copy to si_
      if (collision) {
        Eigen::Vector3d last_position =
            position - step_size_temp*ray_direction;

        if (last_valid.first != nullptr) {
          ompl::base::ScopedState<ompl::mav::StateSpace> last_valid_state(
              si_->getStateSpace());
          last_valid_state->values[0] = last_position.x();
          last_valid_state->values[1] = last_position.y();
          last_valid_state->values[2] = last_position.z();

          si_->copyState(last_valid.first, last_valid_state.get());
        }

        last_valid.second = static_cast<double>(
            (last_position - start).norm()/ray_length);
        return false;
      }

      // update position with dynamic step size
      step_size_temp = std::max(std::min(4.0, remaining_distance), 1e-2);
      position = position + step_size_temp*ray_direction;
    }

    // additionally check goal position
    collision = validity_checker_->checkCollisionWithRobot(goal);
    if (collision) {
      if (last_valid.first != nullptr) {
        ompl::base::ScopedState<ompl::mav::StateSpace>
            last_valid_state(si_->getStateSpace());
        last_valid_state->values[0] = position.x();
        last_valid_state->values[1] = position.y();
        last_valid_state->values[2] = position.z();

        si_->copyState(last_valid.first, last_valid_state.get());
      }

      last_valid.second = static_cast<double>(
          (position - start).norm()/ray_length);
      return false;
    }
    return true;
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So opposite of checkCollision, but same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  virtual bool checkMotion(const base::State* s1, const base::State* s2,
                           std::pair<base::State*, double>& last_valid) const {
    Eigen::Vector3d start = omplToEigen(s1);
    Eigen::Vector3d goal = omplToEigen(s2);
    double voxel_size = validity_checker_->voxel_size();

    voxblox::Point start_scaled, goal_scaled;
    voxblox::AlignedVector<voxblox::GlobalIndex> indices;

    // Convert the start and goal to global voxel coordinates.
    // Actually very simple -- just divide by voxel size.
    start_scaled = start.cast<voxblox::FloatingPoint>() / voxel_size;
    goal_scaled = goal.cast<voxblox::FloatingPoint>() / voxel_size;

    voxblox::castRay(start_scaled, goal_scaled, &indices);

    for (size_t i = 0; i < indices.size(); i++) {
      const voxblox::GlobalIndex& global_index = indices[i];


      Eigen::Vector3d pos = global_index.cast<double>() * voxel_size;
      bool collision =
          validity_checker_->checkCollisionWithRobotAtVoxel(global_index);

      if (collision) {
        if (last_valid.first != nullptr) {
          ompl::base::ScopedState<ompl::mav::StateSpace> last_valid_state(
              si_->getStateSpace());
          last_valid_state->values[0] = pos.x();
          last_valid_state->values[1] = pos.y();
          last_valid_state->values[2] = pos.z();

          si_->copyState(last_valid.first, last_valid_state.get());
        }

        last_valid.second = static_cast<double>(i / indices.size());
        return false;
      }
    }

    return true;
  }

 protected:
  typename std::shared_ptr<VoxbloxValidityChecker<VoxelType> >
      validity_checker_;
};

}  // namespace mav
}  // namespace ompl

#endif  // VOXBLOX_RRT_PLANNER_OMPL_OMPL_VOXBLOX_H_
