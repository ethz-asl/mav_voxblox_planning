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

#include <cblox/core/submap_collection.h>

#include "voxblox_rrt_planner/ompl/ompl_types.h"
#include "voxblox_rrt_planner/ompl/ompl_voxblox.h"
#include "voxblox_rrt_planner/ompl/ompl_cblox.h"

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

  void setCbloxCollisionChecking(double robot_radius,
      voxblox::FloatingPoint voxel_size, float truncation_distance,
      std::function<double(const Eigen::Vector3d& position)> map_function) {

    // state validity checker
    std::shared_ptr<CbloxValidityChecker> validity_checker(
        new CbloxValidityChecker(getSpaceInformation(), robot_radius, map_function));
    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));

    // motion validator
    si_->setMotionValidator(
        base::MotionValidatorPtr(new CbloxMotionValidator(getSpaceInformation(),
            validity_checker, voxel_size, truncation_distance)));
  }

  void constructPrmRoadmap(double num_seconds_to_construct) {
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(num_seconds_to_construct);

    std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
        ->constructRoadmap(ptc);
  }

  void getPrmRoadmap() {
    ROS_INFO_STREAM("[MavSetup] getting roadmap");
    const ompl::geometric::PRM::Graph& roadmap_graph =
        std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
            ->getRoadmap();
//    bool prm_edges = roadmap_graph.m_edges;
//    bool prm_vertices = roadmap_graph.m_vertices;
    auto it_pair = boost::vertices(roadmap_graph);
    ROS_INFO_STREAM("[MavSetup] 1");
    auto it = it_pair.first;
    ROS_INFO_STREAM("[MavSetup] 2");
    const auto it_end = it_pair.second;
    ROS_INFO_STREAM("[MavSetup] 3");
    int count = 0;
    while (it != it_end) {
//      bool test = state(it);
      ompl::base::State* state;
      ROS_INFO_STREAM("[MavSetup] 4");
      boost::get(state, *it);
      ROS_INFO_STREAM("[MavSetup] 5");
      if(state == nullptr) {
        ROS_INFO_STREAM("[MavSetup] empty state");
      } else {
        ROS_INFO_STREAM("[MavSetup] filled state");
      }
      const ompl::base::RealVectorStateSpace::StateType* state_new =
          static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
      ROS_INFO_STREAM("[MavSetup] 6");
      Eigen::Vector3d state_eigen = omplToEigen(state);
      ROS_INFO_STREAM("[MavSetup] 7");
      ROS_INFO_STREAM(count << ": " << state_eigen.transpose());
      it++;
      count++;
    }
    auto edge_pair = boost::edges(roadmap_graph);
    auto edge = edge_pair.first;
    const auto edge_end = edge_pair.second;
    count = 0;
    while (edge != edge_end) {
      ROS_INFO_STREAM(count << ": " << source(*edge, roadmap_graph) << "-"
                                    << target(*edge, roadmap_graph));
      it++;
      count++;
    }
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
