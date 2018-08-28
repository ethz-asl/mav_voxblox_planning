#ifndef MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
#define MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

namespace mav_planning_rviz {

class PlanningInteractiveMarkers {
 public:
  PlanningInteractiveMarkers(const ros::NodeHandle& nh);
  ~PlanningInteractiveMarkers() {}

  void setFrameId(const std::string& frame_id);

  void initialize();

  void enableSetPosetMarker(const mav_msgs::EigenTrajectoryPoint& pose);
  void disableSetPoseMarker();

 private:
  // Creates markers without adding them to the marker server.
  void createMarkers();

  // ROS stuff.
  ros::NodeHandle nh_;
  interactive_markers::InteractiveMarkerServer marker_server_;

  // Settings.
  std::string frame_id_;

  // State.
  bool initialized_;
  visualization_msgs::InteractiveMarker set_pose_marker_;
  // Also have start, goal, and waypoints markers. :)
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
