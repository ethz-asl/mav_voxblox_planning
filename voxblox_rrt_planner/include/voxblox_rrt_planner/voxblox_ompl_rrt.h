#ifndef VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_
#define VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include "voxblox_rrt_planner/ompl/mav_setup_voxblox.h"
#include "voxblox_rrt_planner/ompl_rrt.h"

namespace mav_planning {

class VoxbloxOmplRrt : public BloxOmplRrt {
 public:
  VoxbloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~VoxbloxOmplRrt() {}

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);

  // Only call this once, only call this after setting all settings correctly.
  void setupProblem();

 protected:
  // Setup the problem in OMPL.
  ompl::mav::MavSetupVoxblox problem_setup_voxblox_;

  // NON-OWNED pointers to the relevant layers. TSDF only used if optimistic,
  // ESDF only used if pessimistic.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_
