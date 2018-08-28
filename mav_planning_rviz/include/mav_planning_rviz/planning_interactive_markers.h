#ifndef MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
#define MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_

#include <functional>

#include <interactive_markers/interactive_marker_server.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

namespace mav_planning_rviz {

class PlanningInteractiveMarkers {
 public:
  typedef std::function<void(const mav_msgs::EigenTrajectoryPoint& pose)>
      PoseUpdatedFunctionType;

  PlanningInteractiveMarkers(const ros::NodeHandle& nh);
  ~PlanningInteractiveMarkers() {}

  void setFrameId(const std::string& frame_id);
  // Bind callback for whenever pose updates.

  void setPoseUpdatedCallback(const PoseUpdatedFunctionType& function) {
    pose_updated_function_ = function;
  }

  void initialize();

  // Functions to interface with the set_pose marker:
  void enableSetPoseMarker(const mav_msgs::EigenTrajectoryPoint& pose);
  void disableSetPoseMarker();
  void setPose(const mav_msgs::EigenTrajectoryPoint& pose);

  // Functions to interact with markers from the marker map (no controls):
  void enableMarker(const std::string& id,
                    const mav_msgs::EigenTrajectoryPoint& pose);
  void updateMarkerPose(const std::string& id,
                        const mav_msgs::EigenTrajectoryPoint& pose);
  void disableMarker(const std::string& id);

  void processSetPoseFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

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

  // This is map for waypoint visualization markers:
  std::map<std::string, visualization_msgs::InteractiveMarker> marker_map_;
  // This determines how the markers in the marker map will look:
  visualization_msgs::InteractiveMarker marker_prototype_;

  // State:
  PoseUpdatedFunctionType pose_updated_function_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
