#ifndef VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_
#define VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_

#include <loco_planner/loco.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox/core/common.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/planning_utils.h>

namespace mav_planning {

class VoxbloxLocoPlanner {
 public:
  static constexpr int kN = 10;

  VoxbloxLocoPlanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  // MUST be called to associate the map with the planner.
  void setEsdfMap(const std::shared_ptr<voxblox::EsdfMap>& esdf_map);

  // Callbacks to bind to loco.
  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;
  double getMapDistanceAndGradientVector(const Eigen::VectorXd& position,
                                         Eigen::VectorXd* gradient) const;

  // Evaluate what we've got here.
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Settings for physical constriants.
  PhysicalConstraints constraints_;

  // General settings.
  bool verbose_;
  bool visualize_;
  std::string frame_id_;

  // Planner.
  loco_planner::Loco<kN> loco_;

  // Map.
  std::shared_ptr<voxblox::EsdfMap> esdf_map_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_LOCO_PLANNER_VOXBLOX_LOCO_PLANNER_H_
