#ifndef VOXBLOX_SKELETON_PLANNER_SKELETON_GRAPH_PLANNER_H_
#define VOXBLOX_SKELETON_PLANNER_SKELETON_GRAPH_PLANNER_H_

#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/sparse_graph_planner.h>
#include <voxblox_planning_common/path_shortening.h>

namespace mav_planning {

class SkeletonGraphPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SkeletonGraphPlanner(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private);
  virtual ~SkeletonGraphPlanner() {}

  double getRobotRadius() const { return robot_radius_; }
  void setRobotRadius(double robot_radius);

  bool getShortenPath() const { return shorten_path_; }
  void setShortenPath(bool shorten_path) { shorten_path_ = shorten_path; }

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);
  void setSkeletonLayer(voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer);
  void setSparseGraph(voxblox::SparseSkeletonGraph* sparse_graph);

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* solution);

  // Utilities...
  void convertCoordinatePathToPath(
      const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path) const;

  void shortenPath(const mav_msgs::EigenTrajectoryPoint::Vector& path_in,
                   mav_msgs::EigenTrajectoryPoint::Vector* path_out) const;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  double robot_radius_;
  bool verbose_;
  bool shorten_path_;

  voxblox::SkeletonAStar skeleton_planner_;
  voxblox::SparseGraphPlanner sparse_graph_planner_;
  EsdfPathShortener path_shortener_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_SKELETON_PLANNER_SKELETON_GRAPH_PLANNER_H_
