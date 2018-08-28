#include <mav_visualization/hexacopter_marker.h>

#include "mav_planning_rviz/planning_interactive_markers.h"

namespace mav_planning_rviz {

PlanningInteractiveMarkers::PlanningInteractiveMarkers(
    const ros::NodeHandle& nh)
    : nh_(nh),
      marker_server_("planning_markers"),
      frame_id_("odom"),
      initialized_(false) {}

void PlanningInteractiveMarkers::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
}

void PlanningInteractiveMarkers::initialize() {
  createMarkers();
  initialized_ = true;
}

void PlanningInteractiveMarkers::createMarkers() {
  // First we set up the set point marker.
  set_pose_marker_.header.frame_id = frame_id_;
  set_pose_marker_.name = "set_pose";
  set_pose_marker_.scale = 1.0;
  set_pose_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_pose_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.name = "rotate_yaw";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move z";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = kSqrt2Over2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = kSqrt2Over2;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y";
  set_pose_marker_.controls.push_back(control);

  mav_visualization::HexacopterMarker hm;
  hm.getMarkers(control.markers, set_pose_marker_.scale, false);
  control.always_visible = true;
  set_pose_marker_.controls.push_back(control);
}

void PlanningInteractiveMarkers::enableSetPosetMarker(
    const mav_msgs::EigenTrajectoryPoint& pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;

  marker_server_.insert(set_pose_marker_);
  marker_server_.setCallback(
      set_pose_marker_.name,
      boost::bind(&PlanningInteractiveMarkers::processSetPoseFeedback, this,
                  _1));
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableSetPoseMarker() {
  marker_server_.erase(set_pose_marker_.name);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::setPose(
    const mav_msgs::EigenTrajectoryPoint& pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;
  marker_server_.setPose(set_pose_marker_.name, set_pose_marker_.pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::processSetPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (pose_updated_function_) {
      mav_msgs::EigenTrajectoryPoint pose;
      mav_msgs::eigenTrajectoryPointFromPoseMsg(feedback->pose, &pose);
      pose_updated_function_(pose);
    }
  }

  marker_server_.applyChanges();
}

}  // end namespace mav_planning_rviz
