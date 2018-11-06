#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_local_planner/mav_local_planner.h"

namespace mav_planning {

MavLocalPlanner::MavLocalPlanner(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      command_publishing_spinner_(1, &command_publishing_queue_),
      planning_spinner_(1, &planning_queue_),
      verbose_(false),
      global_frame_id_("map"),
      local_frame_id_("odom"),
      mpc_prediction_horizon_(300),
      command_publishing_dt_(1.0),
      replan_dt_(1.0),
      replan_lookahead_sec_(0.1),
      use_obstacle_avoidance_(true),
      autostart_(true),
      current_waypoint_(-1),
      path_index_(0),
      max_failures_(5),
      num_failures_(0),
      esdf_server_(nh_, nh_private_),
      loco_planner_(nh_, nh_private_) {
  // Set up some settings.
  constraints_.setParametersFromRos(nh_private_);
  esdf_server_.setTraversabilityRadius(constraints_.robot_radius);
  loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
  nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
  nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                    mpc_prediction_horizon_);
  nh_private_.param("replan_dt", replan_dt_, replan_dt_);
  nh_private_.param("replan_lookahead_sec", replan_lookahead_sec_,
                    replan_lookahead_sec_);
  nh_private_.param("command_publishing_dt", command_publishing_dt_,
                    command_publishing_dt_);
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
      "start", &MavLocalPlanner::startCallback, this);
  pause_srv_ = nh_private_.advertiseService(
      "pause", &MavLocalPlanner::pauseCallback, this);
  stop_srv_ = nh_private_.advertiseService(
      "stop", &MavLocalPlanner::stopCallback, this);

  // Start the planning timer. Will no-op most cycles.
  ros::TimerOptions timer_options(
      ros::Duration(replan_dt_),
      boost::bind(&MavLocalPlanner::planningTimerCallback, this, _1),
      &planning_queue_);

  planning_timer_ = nh_.createTimer(timer_options);

  // Start the command publishing spinner.
  command_publishing_spinner_.start();
  planning_spinner_.start();

  // Set up yaw policy.
  yaw_policy_.setPhysicalConstraints(constraints_);
  yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector);
}

void MavLocalPlanner::odometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
}

void MavLocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a waypoint!");
  // TODO(helenol): What do we do with clearing????
  // Cancel any previous trajectory on getting a new one.
  clearTrajectory();

  mav_msgs::EigenTrajectoryPoint waypoint;
  eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

  waypoints_.clear();
  waypoints_.push_back(waypoint);
  current_waypoint_ = 0;

  // Execute one planning step on main thread.
  planningStep();
  startPublishingCommands();
}

void MavLocalPlanner::waypointListCallback(
    const geometry_msgs::PoseArray& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a waypoint list! UNIMPLEMENTED.");
}

void MavLocalPlanner::planningTimerCallback(const ros::TimerEvent& event) {
  // Wait on the condition variable from the publishing...
  if (should_replan_.wait_for(replan_dt_)) {
    planningStep();
  }
}

void MavLocalPlanner::planningStep() {
  // TODO!!!!
  // This is only the first case: plan toward a single waypoint.
  // This will get replaced with logic about doing local planning toward
  // waypoint, or path smoothing, or path repair.
  if (current_waypoint_ < 0 ||
      static_cast<int>(waypoints_.size()) <= current_waypoint_) {
    return;
  }

  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  const double kCloseEnough = 0.05;

  const int64_t kDtNs =
      mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

  ROS_INFO_STREAM("[Mav Local Planner][Plan Step] Current odometry: "
                  << odometry_.position_W.transpose()
                  << " Tracking waypoint: " << waypoint.position_W.transpose());

  // Save success and Trajectory.
  mav_trajectory_generation::Trajectory trajectory;
  bool success = false;

  mav_trajectory_generation::timing::MiniTimer timer;

  if (!path_queue_.empty() && path_index_ < path_queue_.size()) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    ROS_INFO(
        "[Mav Local Planner][Plan Step] Trying to replan on existing path.");
    mav_msgs::EigenTrajectoryPointVector path_chunk;
    size_t replan_start_index;
    {
      replan_start_index =
          std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
                                                     constraints_.sampling_dt),
                   path_queue_.size());

      ROS_INFO(
          "Path queue size: %zu Path queue index: %zu Replan start index: %zu "
          "Lookahead should be: %zu",
          path_queue_.size(), path_index_, replan_start_index,
          static_cast<size_t>((replan_lookahead_sec_) /
                              constraints_.sampling_dt));
      // Cut out the remaining snippet of the trajectory so we can do something
      // with it.
      std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
                std::back_inserter(path_chunk));
      ROS_INFO("path chunk length: %zu", path_chunk.size());
      if (path_chunk.size() == 0) {
        path_chunk.push_back(path_queue_.back());
      }
    }

    bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
    ROS_INFO(
        "[Mav Local Planner][Plan Step] Existing chunk is collision free? %d",
        path_chunk_collision_free);
    // Check if the current path queue goes to the goal and is collision free.
    if ((path_chunk.back().position_W - waypoint.position_W).norm() <
        kCloseEnough) {
      // Collision check the remaining chunk of the trajectory.
      if (path_chunk_collision_free) {
        ROS_INFO(
            "[Mav Local Planner][Plan Step] Current plan is valid, just "
            "rollin' with it.");
        nextWaypoint();
        return;
      }
    }
    // Otherwise we gotta replan this thing anyway.
    success = loco_planner_.getTrajectoryTowardGoal(path_chunk.front(),
                                                    waypoint, &trajectory);
    if (!success) {
      if (path_chunk_collision_free) {
        ROS_INFO(
            "[Mav Local Planner][Plan Step] Couldn't find a solution :( "
            "Continuing existing solution.");
      } else {
        ROS_INFO(
            "[Mav Local Planner][Plan Step] ABORTING! No local solution "
            "found.");
        abort();
        num_failures_++;
        if (num_failures_ > max_failures_) {
          current_waypoint_ = -1;
        }
      }
      return;
    } else {
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);

      ROS_INFO("[Mav Local Planner][Plan Step] Appending new path chunk.");
      if (trajectory.getMaxTime() <= 1e-6) {
        nextWaypoint();
      } else {
        num_failures_ = 0;
        mav_msgs::EigenTrajectoryPointVector new_path_chunk;
        mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, constraints_.sampling_dt, &new_path_chunk);

        retimeTrajectoryWithStartTimeAndDt(
            path_chunk.front().time_from_start_ns, kDtNs, &new_path_chunk);

        new_path_chunk.front().orientation_W_B =
            path_chunk.front().orientation_W_B;
        yaw_policy_.applyPolicyInPlace(&new_path_chunk);

        // Remove what was in the trajectory before.
        if (replan_start_index < path_queue_.size()) {
          path_queue_.erase(path_queue_.begin() + replan_start_index,
                            path_queue_.end());
        }
        ROS_INFO(
            "Starting new path chunk at %f, matching time is %f, cut length: "
            "%zu, path chunk length: %zu",
            path_chunk.front().time_from_start_ns * 1e-9,
            path_queue_.back().time_from_start_ns * 1e-9, path_queue_.size(),
            path_chunk.size());

        ROS_INFO(
            "Path chunk velocity: %f Previous velocity: %f",
            path_chunk.front().velocity_W.x(),
            path_queue_.back().velocity_W.x());
        // Stick the new one in.
        path_queue_.insert(path_queue_.end(), new_path_chunk.begin(),
                           new_path_chunk.end());
        ROS_INFO(
            "New time: %f, new size: %zu",
            (path_queue_.begin() + replan_start_index)->time_from_start_ns *
                1e-9,
            path_queue_.size());
      }
    }
  } else {
    ROS_INFO("[Mav Local Planner][Plan Step] Trying to plan from scratch.");

    // There's nothing planned so far! So we plan from the current odometry.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
                                                    &trajectory);
    ROS_INFO("[Mav Local Planner][Plan Step] Planning success? %d", success);

    if (success) {
      if (trajectory.getMaxTime() <= 1e-6) {
        nextWaypoint();
      } else {
        // Copy this straight into the queue.
        num_failures_ = 0;
        // std::lock_guard<std::recursive_mutex> guard(path_mutex_);
        path_queue_.clear();
        mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, constraints_.sampling_dt, &path_queue_);
        path_queue_.front().orientation_W_B = odometry_.orientation_W_B;
        yaw_policy_.applyPolicyInPlace(&path_queue_);
        path_index_ = 0;
      }
    } else {
      num_failures_++;
      if (num_failures_ > max_failures_) {
        current_waypoint_ = -1;
      }
    }
  }
  ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
           timer.stop());
  visualizePath();
}

void MavLocalPlanner::nextWaypoint() {
  if (waypoints_.size() <= static_cast<size_t>(current_waypoint_)) {
    current_waypoint_ = -1;
  } else {
    current_waypoint_++;
  }
}

void MavLocalPlanner::startPublishingCommands() {
  // Call the service call to takeover publishing commands.
  if (position_hold_client_.exists()) {
    std_srvs::Empty empty_call;
    position_hold_client_.call(empty_call);
  }

  // Publish the first set immediately, on this thread.
  commandPublishTimerCallback(ros::TimerEvent());

  // Need advanced timer options to assign callback queue to this timer.
  ros::TimerOptions timer_options(
      ros::Duration(command_publishing_dt_),
      boost::bind(&MavLocalPlanner::commandPublishTimerCallback, this, _1),
      &command_publishing_queue_);

  command_publishing_timer_ = nh_.createTimer(timer_options);
}

void MavLocalPlanner::commandPublishTimerCallback(
    const ros::TimerEvent& event) {
  constexpr size_t kQueueBuffer = 0;
  if (path_index_ < path_queue_.size()) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    size_t number_to_publish = std::min<size_t>(
        std::floor(command_publishing_dt_ / constraints_.sampling_dt),
        path_queue_.size() - path_index_);

    size_t starting_index = 0;
    if (path_index_ != 0) {
      starting_index = path_index_ + kQueueBuffer;
      if (starting_index >= path_queue_.size()) {
        starting_index = path_index_;
      }
    }

    size_t number_to_publish_with_buffer = std::min<size_t>(
        number_to_publish + mpc_prediction_horizon_ - kQueueBuffer,
        path_queue_.size() - starting_index);

    // TODO(helenol): do this without copy! Use iterators properly!
    mav_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
        path_queue_.begin() + starting_index;
    mav_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
        first_sample + number_to_publish_with_buffer;
    mav_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
                                                               last_sample);

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msg.header.frame_id = local_frame_id_;
    msg.header.stamp = ros::Time::now();

    ROS_INFO(
        "[Mav Local Planner][Command Publish] Publishing %zu samples of %zu. "
        "Start index: %zu Time: %f Start position: %f Start velocity: %f End "
        "time: %f End position: %f",
        trajectory_to_publish.size(), path_queue_.size(), starting_index,
        trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
        trajectory_to_publish.front().position_W.x(),
        trajectory_to_publish.front().velocity_W.x(),
        trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
        trajectory_to_publish.back().position_W.x());
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_to_publish, &msg);

    command_pub_.publish(msg);
    path_index_ += number_to_publish;
    should_replan_.notify();
  }
  // Does there need to be an else????
}

void MavLocalPlanner::abort() {
  // No need to check anything on stop, just clear all the paths.
  clearTrajectory();
  // Make sure to clear the queue in the controller as well (we send about a
  // second of trajectories ahead).
  sendCurrentPose();
}

void MavLocalPlanner::clearTrajectory() {
  std::lock_guard<std::recursive_mutex> guard(path_mutex_);
  command_publishing_timer_.stop();
  path_queue_.clear();
  path_index_ = 0;
}

void MavLocalPlanner::sendCurrentPose() {
  // Sends the current pose with velocity 0 to the controller to clear the
  // controller's trajectory queue.
  // More or less an abort operation.
  mav_msgs::EigenTrajectoryPoint current_point;
  current_point.position_W = odometry_.position_W;
  current_point.orientation_W_B = odometry_.orientation_W_B;

  trajectory_msgs::MultiDOFJointTrajectory msg;
  msgMultiDofJointTrajectoryFromEigen(current_point, &msg);
  command_pub_.publish(msg);
}

bool MavLocalPlanner::startCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  if (path_queue_.size() <= path_index_) {
    ROS_WARN("Trying to start an empty or finished trajectory queue!");
    return false;
  }
  startPublishingCommands();
  return true;
}

bool MavLocalPlanner::pauseCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  if (path_queue_.size() <= path_index_) {
    ROS_WARN("Trying to pause an empty or finished trajectory queue!");
    return false;
  }
  command_publishing_timer_.stop();
  return true;
}

bool MavLocalPlanner::stopCallback(std_srvs::Empty::Request& request,
                                   std_srvs::Empty::Response& response) {
  abort();
  return true;
}

void MavLocalPlanner::visualizePath() {
  // TODO: Split trajectory into two chunks: before and after.
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker path_marker;
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    path_marker = createMarkerForPath(path_queue_, local_frame_id_,
                                      mav_visualization::Color::Black(),
                                      "local_path", 0.025);
  }
  marker_array.markers.push_back(path_marker);
  path_marker_pub_.publish(marker_array);
}

double MavLocalPlanner::getMapDistance(const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

bool MavLocalPlanner::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius - 0.1) {
      return false;
    }
  }
  return true;
}

bool MavLocalPlanner::isPathFeasible(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max + 1e-2) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max + 1e-2) {
      return false;
    }
  }
  return true;
}

}  // namespace mav_planning
