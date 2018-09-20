#ifndef VOXBLOX_RRT_PLANNER_OMPL_OMPL_TYPES_H_
#define VOXBLOX_RRT_PLANNER_OMPL_OMPL_TYPES_H_

#include <ompl/base/spaces/SE3StateSpace.h>
#include <Eigen/Core>

namespace ompl {
namespace mav {

// R3 for position. Other planners take care of orientation.
typedef base::RealVectorStateSpace StateSpace;

inline Eigen::Vector3d omplToEigen(const base::State* state) {
  const mav::StateSpace::StateType* derived =
      static_cast<const mav::StateSpace::StateType*>(state);
  Eigen::Vector3d eigen_state(derived->values[0], derived->values[1],
                              derived->values[2]);

  return eigen_state;
}

}  // namespace mav
}  // namespace ompl

#endif  // VOXBLOX_RRT_PLANNER_OMPL_OMPL_TYPES_H_
