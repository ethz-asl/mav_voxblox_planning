#ifndef VOXBLOX_RRT_PLANNER_OMPL_OMPL_CBLOX_H_
#define VOXBLOX_RRT_PLANNER_OMPL_OMPL_CBLOX_H_

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>

#include <cblox/core/submap_collection.h>

#include "voxblox_rrt_planner/ompl/ompl_types.h"

namespace ompl {
namespace mav {

// c-blox version
class CbloxValidityChecker : public base::StateValidityChecker {
  // todo: getDistance fix einbinden, kann als gegeben angenommen werden!
  // type definitions
  typedef std::function<double(const Eigen::Vector3d& position)>
      MapDistanceFunctionType;

 public:
  CbloxValidityChecker(const base::SpaceInformationPtr& space_info,
      double robot_radius, const MapDistanceFunctionType& function)
        : base::StateValidityChecker(space_info),
          robot_radius_(robot_radius) {
    get_map_distance_ = function;
//    ROS_INFO("[CbloxValidityChecker] initializing");
  }

  virtual bool isValid(const base::State* state) const {
    Eigen::Vector3d robot_position = omplToEigen(state);
//    ROS_INFO("[CbloxValidityChecker] checking state validity");
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
      const Eigen::Vector3d& robot_position) const {
//    ROS_INFO("[CbloxValidityChecker] checking for collision");
    double distance = get_map_distance_(robot_position);
    return robot_radius_ >= distance;
  }

  double remainingDistanceToCollision(const Eigen::Vector3d& position) {
    return get_map_distance_(position) - robot_radius_;
  }

 protected:
  double robot_radius_;

  // function to get map distance
  MapDistanceFunctionType get_map_distance_;
};

// Motion validator that uses either of the validity checkers above to
// validate motions at voxel resolution.
class CbloxMotionValidator : public base::MotionValidator {
 public:
  CbloxMotionValidator(const base::SpaceInformationPtr& space_info,
      const typename std::shared_ptr<CbloxValidityChecker>& validity_checker,
      float voxel_size, float truncation_distance)
      : base::MotionValidator(space_info),
        validity_checker_(validity_checker),
        voxel_size_(voxel_size),
        truncation_distance_(truncation_distance) {
    CHECK(validity_checker);
//    ROS_INFO("[CbloxMotionValidator] initializing");
  }

  bool checkMotion(const base::State* s1, const base::State* s2) const {
    std::pair<base::State*, double> unused;
//    ROS_INFO("[CbloxMotionValidator] checking motion (wrapper)");
    return checkMotion(s1, s2, unused);
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So opposite of checkCollision, but same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  bool checkMotion(const base::State* s1, const base::State* s2,
                   std::pair<base::State*, double>& last_valid) const {
//    ROS_INFO("[CbloxMotionValidator] checking motion");
    Eigen::Vector3d start = omplToEigen(s1);
    Eigen::Vector3d goal = omplToEigen(s2);

    // cast ray from start to finish
    Eigen::Vector3d ray_direction = (goal - start).normalized();
    double ray_length = (goal-start).norm();
    double step_size = voxel_size_ / 2;
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
      step_size_temp = std::max(std::min(
          static_cast<double>(truncation_distance_), remaining_distance), 1e-2);
      position = position + step_size_temp*ray_direction;
    }

    // additionally check goal position
    collision = validity_checker_->checkCollisionWithRobot(goal);
    if (collision) {
//      ROS_INFO("[CbloxMotionValidator] collision at goal point detected");
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

 protected:
  std::shared_ptr<CbloxValidityChecker> validity_checker_;
  float voxel_size_;
  float truncation_distance_;
};

}  // namespace mav
}  // namespace ompl

#endif  // VOXBLOX_RRT_PLANNER_OMPL_OMPL_CBLOX_H_
