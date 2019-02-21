#ifndef VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_H_
#define VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_H_

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "voxblox_rrt_planner/ompl/ompl_types.h"
#include "voxblox_rrt_planner/ompl/ompl_voxblox.h"

namespace ompl {
namespace mav {

// Setup class for a geometric planning problem with R3 state space.
class MavSetup : public geometric::SimpleSetup {
 public:
  MavSetup() : geometric::SimpleSetup(base::StateSpacePtr(new StateSpace(3))) {}

  // Get some defaults.
  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(
            new ompl::base::PathLengthOptimizationObjective(
                getSpaceInformation())));
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTstar(getSpaceInformation())));
  }

  void setRrtConnect() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTConnect(getSpaceInformation())));
  }

  void setInformedRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::InformedRRTstar(getSpaceInformation())));
  }

  void setBitStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::BITstar(getSpaceInformation())));
  }

  void setPrm() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::PRM(getSpaceInformation())));
  }

  const base::StateSpacePtr& getGeometricComponentStateSpace() const {
    return getStateSpace();
  }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

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

  void constructPrmRoadmap(double num_seconds_to_construct) {
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(num_seconds_to_construct);

    std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
        ->constructRoadmap(ptc);
  }

  // Uses the path simplifier WITHOUT using B-spline smoothing which leads to
  // a lot of issues for us.
  void reduceVertices() {
    if (pdef_) {
      const base::PathPtr& p = pdef_->getSolutionPath();
      if (p) {
        time::point start = time::now();
        geometric::PathGeometric& path =
            static_cast<geometric::PathGeometric&>(*p);
        std::size_t num_states = path.getStateCount();

        reduceVerticesOfPath(path);
        // simplifyTime_ member of the parent class.
        simplifyTime_ = time::seconds(time::now() - start);
        OMPL_INFORM(
            "MavSetup: Vertex reduction took %f seconds and changed from %d to "
            "%d states",
            simplifyTime_, num_states, path.getStateCount());
        return;
      }
    }
    OMPL_WARN("No solution to simplify");
  }

  // Simplification of path without B-splines.
  void reduceVerticesOfPath(geometric::PathGeometric& path) {
    const double max_time = 0.1;
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(max_time);

    // Now just call near-vertex collapsing and reduceVertices.
    if (path.getStateCount() < 3) {
      return;
    }

    // try a randomized step of connecting vertices
    bool try_more = false;
    if (ptc == false) {
      try_more = psk_->reduceVertices(path);
    }

    // try to collapse close-by vertices
    if (ptc == false) {
      psk_->collapseCloseVertices(path);
    }

    // try to reduce verices some more, if there is any point in doing so
    int times = 0;
    while (try_more && ptc == false && ++times <= 5) {
      try_more = psk_->reduceVertices(path);
    }
  }
};

}  // namespace mav
}  // namespace ompl

#endif  // VOXBLOX_RRT_PLANNER_OMPL_MAV_SETUP_H_
