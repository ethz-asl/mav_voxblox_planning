#include "mav_local_planner/mav_local_planner.h"

namespace mav_planning {

MavLocalPlanner::MavLocalPlanner(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      command_publishing_spinner_(1, &command_publishing_queue_),
      verbose_(false),
      global_frame_id_("map"),
      local_frame_id_("odom"),
      mpc_prediction_horizon_(300),
      replan_dt_(1.0),
      use_obstacle_avoidance_(true),
      autostart_(true),
      current_waypoint_(-1),
      path_index_(0),
      esdf_server_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);
  esdf_server_->setTraversabilityRadius(constraints_.robot_radius);

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
  nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
  nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                    mpc_prediction_horizon_);
  nh_private_.param("replan_dt", replan_dt_, replan_dt_);
  nh_private_.param("use_obstacle_avoidance", use_obstacle_avoidance_,
                    use_obstacle_avoidance_);
  nh_private_.param("autostart", autostart_, autostart_);

  // Publishers and subscribers.
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &MavLocalPlanner::odometryCallback, this);
  waypoint_sub_ =
      nh_.subscribe("waypoint", 1, &MavLocalPlanner::waypointCallback, this);
  waypoint_list_sub_ = nh_.subscribe(
      "waypoint_list", 1, &MavLocalPlanner::waypointListCallback, this);

  command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "local_path", 1, true);
  full_trajectory_pub_ =
      nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "full_trajectory", 1, true);

  // Services.
  start_srv_ = nh_private_.advertiseService(
      "start", &LocalPlannerNode::startCallback, this);
  pause_srv_ = nh_private_.advertiseService(
      "pause", &LocalPlannerNode::pauseCallback, this);
  stop_srv_ = nh_private_.advertiseService(
      "stop", &LocalPlannerNode::stopCallback, this);

  // Start the command publishing spinner.
  command_publishing_spinner_.start();
}

}  // namespace mav_planning
