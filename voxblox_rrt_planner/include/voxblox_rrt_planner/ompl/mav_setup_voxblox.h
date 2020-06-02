#ifndef VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_VOXBLOX_H_
#define VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_VOXBLOX_H_

#include "voxblox_rrt_planner/ompl/mav_setup.h"
#include "voxblox_rrt_planner/ompl/ompl_voxblox.h"

namespace ompl {
namespace mav {

// Setup class for a geometric planning problem with R3 state space.
class MavSetupVoxblox : public MavSetup {
 public:
  MavSetupVoxblox() : MavSetup() {}

  void setTsdfVoxbloxCollisionChecking(
      double robot_radius, voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
    std::shared_ptr<TsdfVoxbloxValidityChecker> validity_checker(
        new TsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       tsdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::TsdfVoxel>(
            getSpaceInformation(), validity_checker)));
  }

  void setEsdfVoxbloxCollisionChecking(
      double robot_radius, voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
    std::shared_ptr<EsdfVoxbloxValidityChecker> validity_checker(
        new EsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       esdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::EsdfVoxel>(
            getSpaceInformation(), validity_checker)));
  }
};

}  // namespace mav
}  // namespace ompl

#endif  // VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_VOXBLOX_H_
