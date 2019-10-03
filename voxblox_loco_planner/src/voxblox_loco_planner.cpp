#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/vertex.h>
#include <voxblox/utils/timing.h>

#include "voxblox_loco_planner/voxblox_loco_planner.h"

namespace mav_planning {

VoxbloxLocoPlanner::VoxbloxLocoPlanner(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(false),
      visualize_(true),
      frame_id_("odom"),
      num_segments_(3),
      num_random_restarts_(5),
      random_restart_magnitude_(0.5),
      planning_horizon_m_(4.0),
      use_shotgun_(true),
      use_shotgun_path_(true),
      loco_(kD) {
  constraints_.setParametersFromRos(nh_private_);
  shotgun_.setParametersFromRos(nh_private_);

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("planning_horizon_m", planning_horizon_m_,
                    planning_horizon_m_);
  nh_private_.param("use_shotgun", use_shotgun_, use_shotgun_);
  nh_private_.param("use_shotgun_path", use_shotgun_path_, use_shotgun_path_);

  loco_.setRobotRadius(constraints_.robot_radius);

  double loco_epsilon_inflation = 0.5;
  nh_private_.param("loco_epsilon_inflation", loco_epsilon_inflation,
                    loco_epsilon_inflation);
  loco_.setEpsilon(constraints_.robot_radius + loco_epsilon_inflation);

  // Set up optional shotgun intermediate point selection.
  shotgun_.setPhysicalConstraints(constraints_);
  path_shortener_.setConstraints(constraints_);

  // ROS debug visualization.
  planning_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "planning_markers", 1, true);
}

void VoxbloxLocoPlanner::setEsdfMap(
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map) {
  CHECK(esdf_map);
  esdf_map_ = esdf_map;

  loco_.setDistanceAndGradientFunction(
      std::bind(&VoxbloxLocoPlanner::getMapDistanceAndGradientVector, this,
                std::placeholders::_1, std::placeholders::_2));
  loco_.setMapResolution(esdf_map->voxel_size());

  shotgun_.setEsdfMap(esdf_map);
  path_shortener_.setEsdfLayer(esdf_map->getEsdfLayerPtr());
}

double VoxbloxLocoPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAtPosition(position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double VoxbloxLocoPlanner::getMapDistanceAndGradientVector(
    const Eigen::VectorXd& position, Eigen::VectorXd* gradient) const {
  CHECK_EQ(position.size(), 3);
  if (gradient == nullptr) {
    return getMapDistanceAndGradient(position, nullptr);
  }
  Eigen::Vector3d gradient_3d;
  double distance = getMapDistanceAndGradient(position, &gradient_3d);
  *gradient = gradient_3d;
  return distance;
}

double VoxbloxLocoPlanner::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                   &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

// Evaluate what we've got here.
bool VoxbloxLocoPlanner::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      return false;
    }
  }
  return true;
}

bool VoxbloxLocoPlanner::isPathFeasible(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  const double kExcessMargin = 0.01;
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max + kExcessMargin) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max + kExcessMargin) {
      return false;
    }
  }
  return true;
}

bool VoxbloxLocoPlanner::getTrajectoryBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    const mav_msgs::EigenTrajectoryPointVector& initial_path,
    mav_trajectory_generation::Trajectory* trajectory) {
  CHECK(esdf_map_);

  ROS_DEBUG_STREAM("[Voxblox Loco Planner] Start: "
                   << start.position_W.transpose()
                   << " goal: " << goal.position_W.transpose());

  constexpr double kTotalTimeScale = 1.2;
  double total_time =
      kTotalTimeScale * mav_trajectory_generation::computeTimeVelocityRamp(
                            start.position_W, goal.position_W,
                            constraints_.v_max, constraints_.a_max);

  // Check that start and goal aren't basically the same thing...
  constexpr double kMinTime = 0.01;
  if (total_time < kMinTime) {
    return false;
  }

  // If we're doing hotstarts, need to save the previous d_p.

  // Use initial path if provided, otherwise just plan between goals.
  if (!initial_path.empty() && num_segments_ == initial_path.size() - 1) {
    mav_trajectory_generation::Trajectory traj_initial;
    getInitialTrajectory(initial_path, total_time, &traj_initial);
    loco_.setupFromTrajectory(traj_initial);
  } else {
    loco_.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);
  }
  Eigen::VectorXd x0, x;
  loco_.getParameterVector(&x0);
  x = x0;
  loco_.solveProblem();

  // Check if this path is collision-free.
  constexpr double kCollisionSamplingDt = 0.1;
  mav_msgs::EigenTrajectoryPoint::Vector path;
  bool success = false;
  int i = 0;
  for (i = 0; i < num_random_restarts_; i++) {
    loco_.getTrajectory(trajectory);
    mav_trajectory_generation::sampleWholeTrajectory(
        *trajectory, kCollisionSamplingDt, &path);
    success = isPathCollisionFree(path);
    if (success) {
      // Awesome, collision-free path.
      break;
    }

    // Otherwise let's do some random restarts.
    x = x0 + random_restart_magnitude_ * Eigen::VectorXd::Random(x.size());
    loco_.setParameterVector(x);
    loco_.solveProblem();
  }

  if (success) {
    // TODO(helenol): Retime the trajectory!
  }

  if (verbose_) {
    ROS_INFO("[Voxblox Loco Planner] Found solution (%d) after %d restarts.",
             success, i);
  }
  return success;
}

bool VoxbloxLocoPlanner::getInitialTrajectory(
    const mav_msgs::EigenTrajectoryPoint::Vector& waypoints, double total_time,
    mav_trajectory_generation::Trajectory* trajectory) const {
  mav_trajectory_generation::timing::Timer linear_timer("loco/initial");

  mav_trajectory_generation::PolynomialOptimization<kN> poly_opt(kD);

  int num_vertices = waypoints.size();
  int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::JERK;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_vertices, mav_trajectory_generation::Vertex(kD));

  // Add the first and last.
  vertices.front().makeStartOrEnd(0, derivative_to_optimize);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.front().position_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      waypoints.front().velocity_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      waypoints.front().acceleration_W);
  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.back().position_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      waypoints.back().velocity_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      waypoints.back().acceleration_W);

  // Now do the middle bits.
  size_t j = 1;
  for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
    vertices[j].addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].position_W);
    j++;
  }

  std::vector<double> segment_times(num_vertices - 1,
                                    total_time / (num_vertices - 1));
  poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  if (poly_opt.solveLinear()) {
    poly_opt.getTrajectory(trajectory);
  } else {
    return false;
  }
  linear_timer.Stop();
  return true;
}

bool VoxbloxLocoPlanner::getTrajectoryTowardGoal(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_trajectory_generation::Trajectory* trajectory) {
  CHECK_NOTNULL(trajectory);
  trajectory->clear();
  mav_msgs::EigenTrajectoryPoint start_point = start;
  mav_msgs::EigenTrajectoryPoint goal_point = goal;

  // Check if we're already at the goal!
  const double kGoalReachedRange = esdf_map_->voxel_size();
  if ((goal_point.position_W - start_point.position_W).norm() <
      kGoalReachedRange) {
    if (verbose_) {
      ROS_INFO("[Voxblox Loco Planner] Goal already reached!");
    }
    return true;
  }

  Eigen::Vector3d distance_to_waypoint =
      goal_point.position_W - start_point.position_W;
  double planning_distance =
      (goal_point.position_W - start_point.position_W).norm();
  Eigen::Vector3d direction_to_waypoint = distance_to_waypoint.normalized();

  if (planning_distance > planning_horizon_m_) {
    goal_point.position_W =
        start_point.position_W + planning_horizon_m_ * direction_to_waypoint;
    planning_distance = planning_horizon_m_;
  }

  mav_msgs::EigenTrajectoryPointVector shotgun_path;

  // Try to find an intermediate goal to go to if the edge of the planning
  // horizon is occupied.
  bool goal_found = true;
  if (use_shotgun_) {
    goal_found = findIntermediateGoalShotgun(start_point, goal_point,
                                             &goal_point, &shotgun_path);
    if (verbose_) {
      ROS_INFO("[Shotgun] Found (%d) intermediate goal at %f %f %f", goal_found,
               goal_point.position_W.x(), goal_point.position_W.y(),
               goal_point.position_W.z());
    }
    if ((goal_point.position_W - start_point.position_W).norm() <
        kGoalReachedRange) {
      if (verbose_) {
        ROS_INFO("[Voxblox Loco Planner] Intermediate goal already reached!");
      }
      return true;
    }
  } else if (getMapDistance(goal_point.position_W) <
             constraints_.robot_radius) {
    const double step_size = esdf_map_->voxel_size();
    goal_found =
        findIntermediateGoal(start_point, goal_point, step_size, &goal_point);
  }

  if (!goal_found ||
      (goal_point.position_W - start_point.position_W).norm() <
          kGoalReachedRange) {
    return false;
  }

  // Visualization.
  visualization_msgs::MarkerArray marker_array;
  if (visualize_) {
    mav_msgs::EigenTrajectoryPointVector vis_vector;
    vis_vector.push_back(goal_point);
    marker_array.markers.push_back(createMarkerForWaypoints(
        vis_vector, "odom", mav_visualization::Color::Purple(), "goal", 0.2));
    marker_array.markers.push_back(createMarkerForPath(
        shotgun_path, "odom", mav_visualization::Color::Pink(), "shotgun_path",
        0.1));
  }

  bool success = false;
  mav_msgs::EigenTrajectoryPointVector shortened_path;

  if (use_shotgun_ && use_shotgun_path_) {
    path_shortener_.shortenPath(shotgun_path, &shortened_path);
    // Make sure we have the full state at the start and end.
    shortened_path.front() = start_point;
    shortened_path.back() = goal_point;
    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          shortened_path, "odom", mav_visualization::Color::Purple(),
          "shortened_path", 0.05));
    }
  }
  if (visualize_) {
    planning_marker_pub_.publish(marker_array);
  }

  success = getTrajectoryBetweenWaypoints(start_point, goal_point,
                                          shortened_path, trajectory);

  // TODO(DEBUG)
  if (verbose_) {
    mav_trajectory_generation::timing::Timing::Print(std::cout);
    voxblox::timing::Timing::Print(std::cout);
  }
  return success;
}

bool VoxbloxLocoPlanner::getTrajectoryTowardGoalFromInitialTrajectory(
    double start_time,
    const mav_trajectory_generation::Trajectory& trajectory_in,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_trajectory_generation::Trajectory* trajectory) {
  mav_msgs::EigenTrajectoryPoint start;
  start_time = std::min(trajectory_in.getMaxTime(), start_time);
  bool success = sampleTrajectoryAtTime(trajectory_in, start_time, &start);
  if (!success) {
    return false;
  }
  success = getTrajectoryTowardGoal(start, goal, trajectory);
  const bool attempt_to_use_initial = true;
  if ((!success || trajectory->empty()) && attempt_to_use_initial) {
    // Ok that failed, let's just see if we can get the existing trajectory
    // going.
    mav_msgs::EigenTrajectoryPoint back;
    sampleTrajectoryAtTime(trajectory_in, trajectory_in.getMaxTime(), &back);
    mav_msgs::EigenTrajectoryPointVector empty_path;
    success =
        getTrajectoryBetweenWaypoints(start, back, empty_path, trajectory);
  }
  return success;
}

bool VoxbloxLocoPlanner::findIntermediateGoal(
    const mav_msgs::EigenTrajectoryPoint& start_point,
    const mav_msgs::EigenTrajectoryPoint& goal_point, double step_size,
    mav_msgs::EigenTrajectoryPoint* goal_out) const {
  mav_msgs::EigenTrajectoryPoint new_goal = goal_point;

  Eigen::Vector3d distance_to_waypoint =
      goal_point.position_W - start_point.position_W;
  double planning_distance = distance_to_waypoint.norm();
  Eigen::Vector3d direction_to_waypoint = distance_to_waypoint.normalized();

  bool success = false;
  while (planning_distance >= 0.0) {
    success = getNearestFreeSpaceToPoint(new_goal.position_W, step_size,
                                         &new_goal.position_W);
    if (success) {
      break;
    }
    planning_distance -= step_size;
    new_goal.position_W =
        start_point.position_W + planning_distance * direction_to_waypoint;
  }

  if (success) {
    *goal_out = new_goal;
  }
  return success;
}

bool VoxbloxLocoPlanner::findIntermediateGoalShotgun(
    const mav_msgs::EigenTrajectoryPoint& start_point,
    const mav_msgs::EigenTrajectoryPoint& goal_point,
    mav_msgs::EigenTrajectoryPoint* goal_out,
    mav_msgs::EigenTrajectoryPointVector* path_out) {
  CHECK_NOTNULL(goal_out);
  const int num_particles = 10;
  const int max_steps = 400;

  voxblox::AlignedVector<Eigen::Vector3d> path;
  bool success = shotgun_.shootParticles(
      num_particles, max_steps, start_point.position_W, goal_point.position_W,
      &goal_out->position_W, &path);

  if (success && path_out != nullptr) {
    path_out->clear();
    path_out->reserve(path.size());
    for (const Eigen::Vector3d& position : path) {
      mav_msgs::EigenTrajectoryPoint point;
      point.position_W = position;
      path_out->push_back(point);
    }
  }

  return success;
}

bool VoxbloxLocoPlanner::getNearestFreeSpaceToPoint(
    const Eigen::Vector3d& pos, double step_size,
    Eigen::Vector3d* new_pos) const {
  CHECK(esdf_map_);
  Eigen::Vector3d final_pos = pos;
  double distance = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

  const size_t kMaxIter = 20;
  for (size_t i = 0; i < kMaxIter; i++) {
    double distance = getMapDistanceAndGradient(final_pos, &gradient);
    if (distance >= constraints_.robot_radius) {
      *new_pos = final_pos;
      return true;
    }

    if (gradient.norm() > 1e-6) {
      final_pos += gradient.normalized() * step_size;
    }
  }
  return false;
}

}  // namespace mav_planning
