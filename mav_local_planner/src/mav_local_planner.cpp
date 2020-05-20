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
      avoid_collisions_(true),
      autostart_(true),
      plan_to_start_(true),
      smoother_name_("loco"),
      current_waypoint_(-1),
      path_index_(0),
      max_failures_(5),
      num_failures_(0),
      num_tracking_(0),
      valid_existing_plan_(false),
      esdf_server_(nh_, nh_private_),
      loco_planner_(nh_, nh_private_),
      temporary_goal_(false) {
  getParamsFromRos();
  setupRosCommunication();
  startTimers();
  setupMap();
  setupSmoothers();
}

void MavLocalPlanner::getParamsFromRos() {
  // Set up some settings.
  constraints_.setParametersFromRos(nh_private_);
  goal_selector_.setParametersFromRos(nh_private_);

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
  nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
  nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                    mpc_prediction_horizon_);
  nh_private_.param("replan_dt", replan_dt_, replan_dt_);
  max_failures_ = std::ceil(5.0 / replan_dt_);
  nh_private_.param("replan_lookahead_sec", replan_lookahead_sec_,
                    replan_lookahead_sec_);
  nh_private_.param("command_publishing_dt", command_publishing_dt_,
                    command_publishing_dt_);
  nh_private_.param("avoid_collisions", avoid_collisions_, avoid_collisions_);
  nh_private_.param("autostart", autostart_, autostart_);
  nh_private_.param("plan_to_start", plan_to_start_, plan_to_start_);
  nh_private_.param("smoother_name", smoother_name_, smoother_name_);
}

void MavLocalPlanner::setupRosCommunication() {
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

  position_hold_client_ =
      nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");
}

void MavLocalPlanner::startTimers() {
  // Start the planning timer. Will no-op most cycles.
  ros::TimerOptions timer_options(
      ros::Duration(replan_dt_),
      boost::bind(&MavLocalPlanner::planningTimerCallback, this, _1),
      &planning_queue_);

  planning_timer_ = nh_.createTimer(timer_options);

  // Start the command publishing spinner.
  command_publishing_spinner_.start();
  planning_spinner_.start();
}

void MavLocalPlanner::setupMap() {
  esdf_server_.setTraversabilityRadius(constraints_.robot_radius);
  loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());
  goal_selector_.setTsdfMap(esdf_server_.getTsdfMapPtr());

  // load map to esdf_server_
  std::string map_path;
  nh_private_.param("map_path", map_path, map_path);
  if (!map_path.empty()) {
    ROS_INFO_STREAM("[Mav Local Planner] Loading map from " << map_path);
    bool success = esdf_server_.loadMap(map_path);
    if (!success) {
      ROS_WARN("[Mav Local Planner] Could not load map!");
    } else {
      ROS_INFO("[Mav Local Planner] Loaded map successfully.");
    }
  }
}

void MavLocalPlanner::setupSmoothers() {
  // Set up yaw policy.
  yaw_policy_.setPhysicalConstraints(constraints_);
  yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector);

  // Set up smoothers.
  const double voxel_size = esdf_server_.getEsdfMapPtr()->voxel_size();

  // Straight-line smoother.
  ramp_smoother_.setParametersFromRos(nh_private_);

  // Poly smoother.
  poly_smoother_.setParametersFromRos(nh_private_);
  poly_smoother_.setMinCollisionCheckResolution(voxel_size);
  poly_smoother_.setMapDistanceCallback(
      std::bind(&MavLocalPlanner::getMapDistance, this, std::placeholders::_1));
  poly_smoother_.setOptimizeTime(true);
  poly_smoother_.setSplitAtCollisions(avoid_collisions_);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setDistanceAndGradientFunction(
      std::bind(&MavLocalPlanner::getMapDistanceAndGradient, this,
                std::placeholders::_1, std::placeholders::_2));
  loco_smoother_.setOptimizeTime(true);
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setResampleVisibility(true);
  loco_smoother_.setNumSegments(5);
}

void MavLocalPlanner::odometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
}

void MavLocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a waypoint!");
  // Cancel any previous trajectory on getting a new one.
  abort();

  mav_msgs::EigenTrajectoryPoint waypoint;
  eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

  waypoints_.clear();
  waypoints_.push_back(waypoint);
  current_waypoint_ = 0;
  num_failures_ = 0;
  num_tracking_ = 0;

  // Execute one planning step on main thread.
  planningStep();
  startPublishingCommands();
}

void MavLocalPlanner::waypointListCallback(
    const geometry_msgs::PoseArray& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a list of waypoints, %zu long!",
           msg.poses.size());
  // Cancel any previous trajectory on getting a new one.
  abort();

  waypoints_.clear();

  for (const geometry_msgs::Pose& pose : msg.poses) {
    mav_msgs::EigenTrajectoryPoint waypoint;
    eigenTrajectoryPointFromPoseMsg(pose, &waypoint);
    waypoints_.push_back(waypoint);
  }
  current_waypoint_ = 0;
  num_failures_ = 0;
  num_tracking_ = 0;

  // Execute one planning step on main thread.
  planningStep();
  startPublishingCommands();
}

void MavLocalPlanner::planningTimerCallback(const ros::TimerEvent& event) {
  // Wait on the condition variable from the publishing...
  if (should_replan_.wait_for(replan_dt_)) {
    if (verbose_) {
      ROS_WARN(
          "[Mav Planning Timer] Difference between real and expected: %f Real: "
          "%f Expected: %f Now: %f",
          (event.current_real - event.current_expected).toSec(),
          event.current_real.toSec(), event.current_expected.toSec(),
          ros::Time::now().toSec());
    }

    planningStep();
  }
}

void MavLocalPlanner::planningStep() {
  if (verbose_) {
    ROS_INFO("[Mav Local Planner][Plan Step] "
             "Waypoint %zd/%zd, Path Queue %zd/%zd, Failure %d/%d",
             current_waypoint_, waypoints_.size() - 1,
             path_index_, path_queue_.size() - 1,
             num_failures_, max_failures_);
  }

  if (current_waypoint_ < 0 ||
      static_cast<int>(waypoints_.size()) <= current_waypoint_) {
    // This means that we have planned to the end of the waypoints
    // or we're not planning through any waypoints
    return;
  }
  mav_trajectory_generation::timing::MiniTimer timer;

  // First, easiest case: if we're not avoiding collisions, just use the
  // favorite path smoother. We only do this on the first planning call then
  // ignore all the rest.
  if (!avoid_collisions_) {
    mav_msgs::EigenTrajectoryPointVector waypoints;
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    if (plan_to_start_) {
      waypoints.push_back(current_point);
    }
    waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

    mav_msgs::EigenTrajectoryPointVector path;
    bool success = planPathThroughWaypoints(waypoints, &path);
    ROS_INFO("[Mav Local Planner] Planning through all successful? %d",
             success);
    if (success) {
      replacePath(path);
      current_waypoint_ = waypoints_.size();
    } else {
      ROS_ERROR("[Mav Local Planner] Waypoint planning failed!");
    }

  } else {
    // empty sphere of robot position to be observed and free
    voxblox::utils::emptySphereAroundPoint<voxblox::EsdfVoxel>(
        odometry_.position_W.cast<float>(), constraints_.robot_radius + 0.01,
        constraints_.robot_radius * 2,
        esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr());

    // Get planning position
    getPlanningStart();
    visualizePlanningStart();
    if (verbose_) {
      ROS_INFO("[Mav Local Planner][Plan Step] Replanning at path index %ld with "
               "pose(%.2f %.2f %.2f), velocity %.3f m/s, and "
               "acceleration %.3f m/s^2", planning_index_,
               planning_start_pose_.position_W.x(),
               planning_start_pose_.position_W.y(),
               planning_start_pose_.position_W.z(),
               planning_start_pose_.velocity_W.norm(),
               planning_start_pose_.acceleration_W.norm());
    }

    // Plan
    mav_msgs::EigenTrajectoryPointVector path;
    size_t waypoint_index;
    temporary_waypoints_.clear();
    bool success = findPathThroughCurrentWaypointList(&path, &waypoint_index);
    ROS_INFO("[Mav Local Planner][Plan Step] "
             "Planning of waypoint list successful? %d", success);
    if (success) {
      valid_existing_plan_ = true;
    }

    // In case replanning is not successful, we retry the planning with an
    // initial guess as the current trajectory
    if (!success && !existing_path_chunk_.empty() && valid_existing_plan_) {
      // sample the existing path chunk for temporary waypoints
      sampleExistingPath();

      // Replan
      success = findPathThroughCurrentWaypointList(&path, &waypoint_index);
      ROS_INFO("[Mav Local Planner][Plan Step] "
               "Replanning with existing path successful? %d", success);
    }

    // Use path
    if (success) {
      num_failures_ = 0;
      num_tracking_ = 0;
      current_waypoint_ = waypoint_index;
      insertPath(path);
    } else {
      // check existing path for collisions
      bool existing_path_collision_free = false;
      if (!existing_path_chunk_.empty()) {
        existing_path_collision_free =
            isPathCollisionFree(existing_path_chunk_);
      }

      if (existing_path_collision_free && path_index_ < planning_index_) {
        // Stick with existing path
        ROS_INFO( "[Mav Local Planner][Plan Step] Path chunk (%zd) "
                  "collision free, continuing existing solution.",
                  existing_path_chunk_.size());
      } else {
        // Give up!
        avoidCollisionsTowardWaypoint();
      }
    }
  }

  ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
           timer.stop());
  visualizePath();
}

bool MavLocalPlanner::findPathThroughCurrentWaypointList(
    mav_msgs::EigenTrajectoryPointVector* path,
    size_t* waypoint_index) {
  CHECK_NOTNULL(path);
  CHECK_NOTNULL(waypoint_index);

  // init returned index
  *waypoint_index = current_waypoint_;

  constexpr double kCloseToOdometry = 0.1;

  // Find list of waypoints to plan through
  mav_msgs::EigenTrajectoryPointVector free_waypoints;

  // Check temporary waypoint list for collisions
  for (const mav_msgs::EigenTrajectoryPoint& waypoint : temporary_waypoints_) {
    // skip occupied temporary waypoints
    if (getMapDistance(waypoint.position_W) < constraints_.robot_radius) {
      if (verbose_) {
        ROS_INFO("[Mav Local Planner][Plan List] "
                 "Temporary waypoint at (%.2f %.2f %.2f) in collision"
                 " - %.3f/%.3f",
                 waypoint.position_W.x(), waypoint.position_W.y(),
                 waypoint.position_W.z(), getMapDistance(waypoint.position_W),
                 constraints_.robot_radius);
      }
      continue;
    }
    free_waypoints.emplace_back(waypoint);
  }

  // Check waypoint list for collisions
  for (size_t idx = current_waypoint_; idx < waypoints_.size(); ++idx) {
    const mav_msgs::EigenTrajectoryPoint& waypoint = waypoints_[idx];
    // plan only through free waypoints
    if (getMapDistance(waypoint.position_W) < constraints_.robot_radius) {
      if (verbose_) {
        ROS_INFO("[Mav Local Planner][Plan List] "
                 "Waypoint %ld at (%.2f %.2f %.2f) in collision - %.3f/%.3f",
                 idx, waypoint.position_W.x(), waypoint.position_W.y(),
                 waypoint.position_W.z(),
                 getMapDistance(waypoint.position_W),
                 constraints_.robot_radius);
      }
      break;
    }
    free_waypoints.emplace_back(waypoint);
    *waypoint_index = idx;
  }

  if (free_waypoints.empty()) {
    ROS_WARN("[Mav Local Planner][Plan List] No free waypoints.");
    return false;
  }

  // If the path doesn't ALREADY start near the start, the first waypoint
  // should be the current pose.
  if (plan_to_start_ && (planning_start_pose_.position_W
      - free_waypoints.front().position_W).norm() >
      kCloseToOdometry) {
    // Check, if planning position is on the way from the first
    // to the second waypoint
    if (free_waypoints.size() > 1) {
      Eigen::Vector3d segment =
          free_waypoints[1].position_W - free_waypoints[0].position_W;
      double t = segment.dot(free_waypoints[0].position_W
                                 - planning_start_pose_.position_W);
      t = std::min(1.0, std::max(0.0, t));
      Eigen::Vector3d closest_point =
          free_waypoints[0].position_W + t * segment;
      if ((planning_start_pose_.position_W - closest_point).norm()
          < constraints_.robot_radius) {
        free_waypoints.erase(free_waypoints.begin());
        ROS_WARN("[Mav Local Planner][Plan List] "
                 "Distance from odometry to first path segment: %.3f, "
                 "Removed the first waypoint as redundant!",
                 (planning_start_pose_.position_W - closest_point).norm());
      } else {
        ROS_INFO("[Mav Local Planner][Plan List] "
                 "Distance from odometry to first path segment: %.3f",
                 (planning_start_pose_.position_W - closest_point).norm());
      }
    }

    free_waypoints.insert(free_waypoints.begin(), planning_start_pose_);
    if (getMapDistance(planning_start_pose_.position_W)
        < constraints_.robot_radius) {
      // collision detected
      ROS_INFO_COND(verbose_, "[Mav Local Planner][Plan List] "
                              "Start position at (%.2f %.2f %.2f) in collision"
                              " - %.3f/%.3f",
                    planning_start_pose_.position_W.x(),
                    planning_start_pose_.position_W.y(),
                    planning_start_pose_.position_W.z(),
                    getMapDistance(planning_start_pose_.position_W),
                    constraints_.robot_radius);
      return false;
    }
  }

  // check, if enough waypoints to warrant planning
  if (free_waypoints.size() < 2) {
    ROS_WARN("[Mav Local Planner][Plan List] Too few free waypoints (%zu).",
             free_waypoints.size());
    return false;
  }

  // Resample segments between waypoints for better results
  mav_msgs::EigenTrajectoryPointVector sampled_waypoints;
  ROS_INFO_COND(verbose_, "[Mav Local Planner] Sampling %lu waypoints.",
                free_waypoints.size());
  constexpr double sampling_interval = 2.0;
  for (ulong i = 0; i < free_waypoints.size() - 1; ++i) {
    Eigen::Vector3d segment = free_waypoints[i + 1].position_W
        - free_waypoints[i].position_W;
    int num_points = std::ceil(segment.norm() / sampling_interval);
    Eigen::Vector3d new_segment = segment / num_points;
    mav_msgs::EigenTrajectoryPoint new_waypoint = free_waypoints[i];
    sampled_waypoints.emplace_back(new_waypoint);
    for (int j = 1; j < num_points; ++j) {
      new_waypoint.position_W += new_segment;
      sampled_waypoints.emplace_back(new_waypoint);
    }
  }
  sampled_waypoints.emplace_back(free_waypoints[free_waypoints.size() - 1]);
  ROS_INFO_COND(verbose_, "[Mav Local Planner] Sampled %lu extra waypoints",
                sampled_waypoints.size() - free_waypoints.size());

  // Visualize waypoints
  visualizeWaypoints(sampled_waypoints);

  // Plan through waypoints
  // There is some hope! Maybe we can do path smoothing on these guys.
  bool success = planPathThroughWaypoints(sampled_waypoints, path);
  if (success) {
    success = isPathCollisionFree(*path);
    if (!success) {
      ROS_WARN("[Mav Local Planner][Plan List] "
               "Path was not collision free. :(");
    }
  }
  return success;
}

void MavLocalPlanner::getPlanningStart() {
  // Init
  existing_path_chunk_.clear();
  planning_index_ = path_index_;
  const double kCloseEnough = constraints_.robot_radius/2;
  constexpr double kReached = 0.1;

  // Have we reached the next waypoint?
  // check if we already reached the next waypoint, increase accordingly
  if ((waypoints_[current_waypoint_].position_W
      - odometry_.position_W).norm() < kCloseEnough) {
    if (!nextWaypoint() && (waypoints_[current_waypoint_].position_W
        - odometry_.position_W).norm() < kReached) {
      finishWaypoints();
      return;
    }
  }

  // No existing path: plan from odometry position
  if (path_queue_.empty()) {
    planning_start_pose_.position_W = odometry_.position_W;
    planning_start_pose_.orientation_W_B = odometry_.orientation_W_B;
    planning_start_pose_.velocity_W = odometry_.getVelocityWorld();
    planning_start_pose_.angular_velocity_W =
        odometry_.orientation_W_B * odometry_.angular_velocity_B;
    return;
  }

  // Existing path: plan from position where we will be at after lookahead time
  mav_msgs::EigenTrajectoryPointVector current_path_chunk;
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    planning_index_ =
        std::min(path_index_ + static_cast<size_t>(replan_lookahead_sec_ /
                     constraints_.sampling_dt),
                 path_queue_.size());

    std::copy(path_queue_.begin() + planning_index_, path_queue_.end(),
              std::back_inserter(existing_path_chunk_));
    std::copy(path_queue_.begin(), path_queue_.begin() + planning_index_,
              std::back_inserter(current_path_chunk));
  }

  // Collision check path until replanning.
  if (!isPathCollisionFree(current_path_chunk)) {
    // abort and plan from scratch
    abort();
    existing_path_chunk_.clear();
    planning_index_ = path_index_;
    planning_start_pose_.position_W = odometry_.position_W;
    planning_start_pose_.orientation_W_B = odometry_.orientation_W_B;
    planning_start_pose_.velocity_W = odometry_.getVelocityWorld();
    planning_start_pose_.angular_velocity_W =
        odometry_.orientation_W_B * odometry_.angular_velocity_B;
    return;
  }

  // set position to replan from
  if (existing_path_chunk_.empty()) {
    planning_start_pose_ = path_queue_.back();
  } else {
    planning_start_pose_ = existing_path_chunk_.front();
  }

  // Have we reached the next waypoint?
  // check if we already reached the next waypoint, increase accordingly
  if ((waypoints_[current_waypoint_].position_W
      - planning_start_pose_.position_W).norm() < kCloseEnough) {
    if (!nextWaypoint() && (waypoints_[current_waypoint_].position_W
        - planning_start_pose_.position_W).norm() < kReached) {
      finishWaypoints();
      return;
    }
  }
}

void MavLocalPlanner::avoidCollisionsTowardWaypoint() {
  if (num_tracking_ >= max_failures_) {
    // Maximum tracking attempts reached, aborting
    ROS_WARN("[Mav Local Planner][Next Waypoint] ABORTING! Waypoint has been "
             "unsuccessfully tracked too many times.");
    num_failures_ = max_failures_;
    abort();
    dealWithFailure();
  }
  ++num_tracking_;

  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  if (verbose_) {
    ROS_INFO_STREAM("[Mav Local Planner][Plan Step] Current position at ("
                    << planning_start_pose_.position_W.transpose()
                    << "), Tracking waypoint " << current_waypoint_
                    << " at (" << waypoint.position_W.transpose() << ")");
  }

  // Plan towards waypoint
  mav_trajectory_generation::Trajectory trajectory;
  bool success = loco_planner_.getTrajectoryTowardGoal(planning_start_pose_,
                                                       waypoint, &trajectory);
  ROS_INFO("[Mav Local Planner][Next Waypoint] "
           "Planning towards waypoint %ld successful? %d",
           current_waypoint_, success);

  // Treat very short trajectories
  const double kCloseEnough = constraints_.robot_radius / 2;
  if (trajectory.getMaxTime() <= 0.1) {
    mav_msgs::EigenTrajectoryPointVector new_path_chunk;
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, &new_path_chunk);
    if (!new_path_chunk.empty() and
        (new_path_chunk.back().position_W - waypoint.position_W).norm() <
            kCloseEnough) {
      // Final position close enough to next waypoint,
      // continue in waypoint list
      nextWaypoint();
      success = true;
    } else {
      ROS_WARN("[Mav Local Planner][Next Waypoint] Path was too short.");
      success = false;
    }
  }

  // Replanning towards waypoint was not successful
  if (!success) {
    if (!path_queue_.empty()) {
      ROS_WARN("[Mav Local Planner][Next Waypoint] ABORTING!");
      abort();
    }
    dealWithFailure();
    return;
  }

  // Planning successful
  mav_msgs::EigenTrajectoryPointVector path;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory, constraints_.sampling_dt, &path);

  // Check new plan for collisions
  if (!isPathCollisionFree(path)) {
    ROS_WARN("[Mav Local Planner][Next Waypoint] Path was not collision free.");
    if (!path_queue_.empty()) {
      ROS_WARN("[Mav Local Planner][NextWaypoint] ABORTING!");
      abort();
    }
    dealWithFailure();
    return;
  }

  // Insert new path
  num_failures_ = 0;
  valid_existing_plan_ = false;
  insertPath(path);
}

bool MavLocalPlanner::planPathThroughWaypoints(
    const mav_msgs::EigenTrajectoryPointVector& waypoints,
    mav_msgs::EigenTrajectoryPointVector* path) {
  CHECK_NOTNULL(path);
  bool success = false;
  if (smoother_name_ == "loco") {
    if (waypoints.size() == 2) {
      success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                       waypoints[1], path);
    } else {
      success = loco_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
  } else if (smoother_name_ == "polynomial") {
    success = poly_smoother_.getPathBetweenWaypoints(waypoints, path);

  } else if (smoother_name_ == "ramp") {
    success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
  } else {
    // Default case is ramp!
    ROS_ERROR(
        "[Mav Local Planner] Unknown smoother type %s, using ramp instead.",
        smoother_name_.c_str());
    success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
  }
  return success;
}

void MavLocalPlanner::sampleExistingPath() {
  // sampling interval in meters
  constexpr double interval = 2;

  // sample path chunk
  temporary_waypoints_.clear();
  temporary_waypoints_.emplace_back(planning_start_pose_);
  double distance = 0;
  for (size_t idx = 1; idx < existing_path_chunk_.size(); ++idx) {
    distance += (existing_path_chunk_[idx].position_W
        - existing_path_chunk_[idx - 1].position_W).norm();
    if (distance < interval) {
      continue;
    }

    size_t waypoint_idx = idx;

    // check position for collision
    if (getMapDistance(existing_path_chunk_[idx].position_W)
        < constraints_.robot_radius) {
      // find closest position in path that is collision free
      double offset_forward = 0, offset_backward = 0;
      int max_idx = std::min(existing_path_chunk_.size() - 1 - idx, idx);
      for (int i = 1; i < max_idx + 1; ++i) {
        // check forward direction
        if (getMapDistance(existing_path_chunk_[idx + i].position_W)
            >= constraints_.robot_radius - 0.1) {
          waypoint_idx = idx + i;
          break;
        }
        offset_forward += (existing_path_chunk_[idx + i].position_W
            - existing_path_chunk_[idx + i - 1].position_W).norm();
        if (offset_forward > interval/2) {
          break;
        }

        // check backward direction
        if (getMapDistance(existing_path_chunk_[idx - i].position_W)
            >= constraints_.robot_radius - 0.1) {
          waypoint_idx = idx - i;
          break;
        }
        offset_backward += (existing_path_chunk_[idx - i].position_W
            - existing_path_chunk_[idx - i + 1].position_W).norm();
        if (offset_backward > interval/2) {
          break;
        }
      }
    }
    temporary_waypoints_.emplace_back(existing_path_chunk_[waypoint_idx]);
    distance = 0;
  }
  temporary_waypoints_.erase(temporary_waypoints_.begin());

  // avoid oversampling close to the current next waypoint
  if (!temporary_waypoints_.empty() and
      (temporary_waypoints_.back().position_W -
          waypoints_[current_waypoint_].position_W).norm() <= interval) {
    temporary_waypoints_.erase(temporary_waypoints_.end());
  }
}

bool MavLocalPlanner::nextWaypoint() {
  num_tracking_ = 0;
  valid_existing_plan_ = false;
  if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()) - 1) {
    current_waypoint_ = waypoints_.size() - 1;
    return false;
  } else {
    current_waypoint_++;
    return true;
  }
}

void MavLocalPlanner::finishWaypoints() {
  current_waypoint_ = waypoints_.size();
}

void MavLocalPlanner::insertPath(
    const mav_msgs::EigenTrajectoryPointVector& path) {
  if (!path_queue_.empty()) {
    // Inserting new path chunk into path queue
    mav_msgs::EigenTrajectoryPointVector new_path = path;
    const int64_t kDtNs =
        mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);
    retimeTrajectoryWithStartTimeAndDt(
        planning_start_pose_.time_from_start_ns, kDtNs, &new_path);

    new_path.front().orientation_W_B =
        planning_start_pose_.orientation_W_B;
    yaw_policy_.applyPolicyInPlace(&new_path);

    // Remove what was in the trajectory before.
    if (planning_index_ < path_queue_.size()) {
      path_queue_.erase(path_queue_.begin() + planning_index_,
                        path_queue_.end());
    }
    // Stick the new one in.
    path_queue_.insert(path_queue_.end(), new_path.begin(), new_path.end());
  } else {
    // Copy this straight into the queue.
    replacePath(path);
    startPublishingCommands();
  }
}

void MavLocalPlanner::replacePath(
    const mav_msgs::EigenTrajectoryPointVector& path) {
  std::lock_guard<std::recursive_mutex> guard(path_mutex_);
  path_queue_.clear();
  path_queue_ = path;
  path_queue_.front().orientation_W_B = odometry_.orientation_W_B;
  yaw_policy_.applyPolicyInPlace(&path_queue_);
  path_index_ = 0;
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

    if (verbose_) {
      ROS_INFO("[Mav Local Planner][Command Publish] "
               "Publishing %zu samples of %zu. Start index: %zu\n"
               "Start Time: %.3f, Position: %.2f %.2f %.2f, Velocity: %.2f %.2f %.2f\n"
               "End   Time: %.3f, Position: %.2f %.2f %.2f, Velocity: %.2f %.2f %.2f",
               trajectory_to_publish.size(), path_queue_.size(), starting_index,
               trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
               trajectory_to_publish.front().position_W.x(),
               trajectory_to_publish.front().position_W.y(),
               trajectory_to_publish.front().position_W.z(),
               trajectory_to_publish.front().velocity_W.x(),
               trajectory_to_publish.front().velocity_W.y(),
               trajectory_to_publish.front().velocity_W.z(),
               trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
               trajectory_to_publish.back().position_W.x(),
               trajectory_to_publish.back().position_W.y(),
               trajectory_to_publish.back().position_W.z(),
               trajectory_to_publish.back().velocity_W.x(),
               trajectory_to_publish.back().velocity_W.y(),
               trajectory_to_publish.back().velocity_W.z());
    }
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

void MavLocalPlanner::visualizeWaypoints(
    const mav_msgs::EigenTrajectoryPointVector& waypoints) const {
  // initialize visualization
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = local_frame_id_;
  path_marker.frame_locked = true;
  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::CUBE_LIST;
  path_marker.ns = "local_waypoints";
  path_marker.scale.x = 0.1;
  path_marker.scale.y = path_marker.scale.x;
  path_marker.scale.z = path_marker.scale.x;
  geometry_msgs::Point point_msg;
  mav_visualization::Color color_msg = mav_visualization::Color::Gray();
  color_msg.a = 0.75;
  path_marker.color.a = color_msg.a;

  // visualize sampled waypoint
  for (const mav_msgs::EigenTrajectoryPoint& waypoint : waypoints) {
    mav_msgs::pointEigenToMsg(waypoint.position_W, &point_msg);
    path_marker.points.push_back(point_msg);
    path_marker.colors.push_back(color_msg);
  }

  // publish visualization
  marker_array.markers.push_back(path_marker);
  path_marker_pub_.publish(marker_array);
  ROS_INFO_COND(verbose_, "[Mav Local Planner] Visualized waypoints.");
}

void MavLocalPlanner::visualizePlanningStart() const {
  // initialize message
  visualization_msgs::Marker planning_marker;
  planning_marker.header.frame_id = local_frame_id_;
  planning_marker.frame_locked = true;
  planning_marker.header.stamp = ros::Time::now();
  planning_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  planning_marker.ns = "planning_start_pose";
  planning_marker.scale.x = 0.15;
  planning_marker.scale.y = planning_marker.scale.x;
  planning_marker.scale.z = planning_marker.scale.x;

  mav_visualization::Color color_msg = mav_visualization::Color::Gray();
  color_msg.a = 0.75;
  planning_marker.color = color_msg;
  planning_marker.colors.push_back(color_msg);

  geometry_msgs::Point point_msg;
  mav_msgs::pointEigenToMsg(planning_start_pose_.position_W, &point_msg);
  planning_marker.points.push_back(point_msg);

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(planning_marker);
  path_marker_pub_.publish(marker_array);
}

void MavLocalPlanner::visualizePath() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker path_marker;
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    // Cut out the flown path queue to visualize
    mav_msgs::EigenTrajectoryPointVector path_before;
    std::copy(path_queue_.begin(), path_queue_.begin()
                  + std::min(path_index_ + 1, path_queue_.size()),
              std::back_inserter(path_before));
    path_marker = createMarkerForPath(path_before, local_frame_id_,
                                      mav_visualization::Color::Gray(),
                                      "local_path_before", 0.05);
    path_marker.header.frame_id = local_frame_id_;
    path_marker.frame_locked = true;
    marker_array.markers.push_back(path_marker);
    // Cut out the upcoming path queue to visualize
    mav_msgs::EigenTrajectoryPointVector path_after;
    std::copy(path_queue_.begin() + std::max(path_index_, 0lu),
              path_queue_.end(),
              std::back_inserter(path_after));
    path_marker = createMarkerForPath(path_after, local_frame_id_,
                                      mav_visualization::Color::Black(),
                                      "local_path_after", 0.05);
    path_marker.header.frame_id = local_frame_id_;
    path_marker.frame_locked = true;
    marker_array.markers.push_back(path_marker);
  }
  path_marker_pub_.publish(marker_array);
}

double MavLocalPlanner::getMapDistance(const Eigen::Vector3d& position) const {
  double weight = 0.0;
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_.getTsdfMapPtr()->getWeightAtPosition(
          position, kInterpolate, &weight) || weight < voxblox::kFloatEpsilon
      || !esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    // optimistic behavior at odometry position
    if ((position - odometry_.position_W).norm() < constraints_.robot_radius) {
      return constraints_.robot_radius + 0.1;
    }
    return 0.0;
  }
  return distance;
}

double MavLocalPlanner::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double weight = 0.0;
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_.getTsdfMapPtr()->getWeightAtPosition(
          position, kInterpolate, &weight) || weight < voxblox::kFloatEpsilon
      || !esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    // optimistic behavior at odometry position
    if ((position - odometry_.position_W).norm() < constraints_.robot_radius) {
      *gradient =
          (constraints_.robot_radius - (position - odometry_.position_W).norm())
              * (position - odometry_.position_W).normalized();
      return constraints_.robot_radius + 0.1;
    }
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

bool MavLocalPlanner::dealWithFailure() {
  if (current_waypoint_ < 0) {
    return false;
  }
  should_replan_.notify();

  constexpr double kCloseEnough = 0.05;
  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  mav_msgs::EigenTrajectoryPoint goal = waypoint;
  if (temporary_goal_ &&
      static_cast<int64_t>(waypoints_.size()) > current_waypoint_ + 1) {
    goal = waypoints_[current_waypoint_ + 1];
  }
  mav_msgs::EigenTrajectoryPoint current_point;
  current_point.position_W = odometry_.position_W;
  current_point.orientation_W_B = odometry_.orientation_W_B;

  mav_msgs::EigenTrajectoryPoint current_goal;
  if (!goal_selector_.selectNextGoal(goal, waypoint, current_point,
                                     &current_goal)) {
    num_failures_++;
    ROS_INFO("[Mav Local Planning][Failed] No next goal selected.");
    if (num_failures_ > max_failures_) {
      current_waypoint_ = -1;
      ROS_ERROR("[Mav Local Planning][Failed] Max number of failures reached, "
               "waypoint list abandoned.");
    }
    return false;
  } else {
    if ((current_goal.position_W - waypoint.position_W).norm() < kCloseEnough) {
      // Goal is unchanged. :(
      temporary_goal_ = false;
      ROS_INFO("[Mav Local Planning][Failed] Goal is unchanged.");
      return false;
    } else if ((current_goal.position_W - goal.position_W).norm() <
               kCloseEnough) {
      // This is just the next waypoint that we're trying to go to.
      current_waypoint_++;
      temporary_goal_ = false;
      ROS_INFO("[Mav Local Planning][Failed] "
               "We've reached a waypoint, all good.");
      return true;
    } else {
      // Then this is something different!
      temporary_goal_ = true;
      waypoints_.insert(waypoints_.begin() + current_waypoint_, current_goal);
      ROS_INFO("[Mav Local Planning][Failed] Inserting temporary waypoint.");
      return true;
    }
  }
}

}  // namespace mav_planning
