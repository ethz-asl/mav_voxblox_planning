#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include "rrt_planner_voxblox/rrt_planner_voxblox.h"

namespace mav_planning {

RrtPlannerVoxblox::RrtPlannerVoxblox(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("odom"),
      visualize_(true),
      do_smoothing_(true),
      last_trajectory_valid_(false),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()),
      voxblox_server_(nh_, nh_private_),
      rrt_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);

  std::string input_filepath;
  nh_private_.param("voxblox_path", input_filepath, input_filepath);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  polynomial_trajectory_pub_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
          "polynomial_trajectory", 1);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &RrtPlannerVoxblox::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &RrtPlannerVoxblox::publishPathCallback, this);

  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  if (!input_filepath.empty()) {
    // Verify that the map has an ESDF layer, otherwise generate it.

    if (!voxblox_server_.loadMap(input_filepath)) {
      ROS_ERROR("Couldn't load ESDF map!");

      // Check if the TSDF layer is non-empty...
      if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
        ROS_INFO("Generating ESDF layer from TSDF.");
        // If so, generate the ESDF layer!

        const bool full_euclidean_distance = true;
        voxblox_server_.updateEsdfBatch(full_euclidean_distance);
      } else {
        ROS_ERROR("TSDF map also empty! Check voxel size!");
      }
    }
  }

  ROS_INFO(
      "Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // TODO(helenol): figure out what to do with optimistic/pessimistic here.
  rrt_.setRobotRadius(constraints_.robot_radius);
  rrt_.setOptimistic(false);

  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Figure out map bounds!
  computeMapBounds(&lower_bound_, &upper_bound_);

  // Inflate the bounds a bit.
  constexpr double kBoundInflationMeters = 0.5;
  // Don't in flate in z. ;)
  rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0),
                 upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0));
  rrt_.setupProblem();

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  // Set up the path smoother as well.
  smoother_.setParametersFromRos(nh_private_);
  smoother_.setMapDistanceCallback(std::bind(&RrtPlannerVoxblox::getMapDistance,
                                             this, std::placeholders::_1));

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }
}

bool RrtPlannerVoxblox::publishPathCallback(std_srvs::EmptyRequest& request,
                                            std_srvs::EmptyResponse& response) {
  if (!last_trajectory_valid_) {
    ROS_ERROR("Can't publish trajectory, marked as invalid.");
    return false;
  }

  ROS_INFO("Publishing path.");

  if (!do_smoothing_) {
    geometry_msgs::PoseArray pose_array;
    pose_array.poses.reserve(last_waypoints_.size());
    for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
      geometry_msgs::PoseStamped pose_stamped;
      mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
      pose_array.poses.push_back(pose_stamped.pose);
    }

    pose_array.header.frame_id = frame_id_;
    waypoint_list_pub_.publish(pose_array);
  } else {
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
        last_trajectory_, &msg);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    polynomial_trajectory_pub_.publish(msg);
  }
  return true;
}

void RrtPlannerVoxblox::computeMapBounds(Eigen::Vector3d* lower_bound,
                                         Eigen::Vector3d* upper_bound) const {
  voxblox::BlockIndexList all_blocks;
  tsdf_map_->getTsdfLayerPtr()->getAllAllocatedBlocks(&all_blocks);

  voxblox::Point lower_bound_pt;
  voxblox::Point upper_bound_pt;
  bool first_block = true;

  for (const voxblox::BlockIndex& block_index : all_blocks) {
    voxblox::Block<voxblox::TsdfVoxel>::ConstPtr block =
        tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
    if (first_block) {
      lower_bound_pt = block->origin();
      upper_bound_pt = block->origin().array() + block->block_size();
      first_block = false;
      continue;
    }

    lower_bound_pt = lower_bound_pt.array().min(block->origin().array());
    upper_bound_pt = upper_bound_pt.array().max(block->origin().array() +
                                                block->block_size());
  }

  *lower_bound = lower_bound_pt.cast<double>();
  *upper_bound = upper_bound_pt.cast<double>();
}

bool RrtPlannerVoxblox::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  ROS_INFO("Planning path.");

  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    return false;
  }
  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    return false;
  }

  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

  bool success =
      rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
  rrtstar_timer.Stop();
  double path_length = computePathLength(waypoints);
  int num_vertices = waypoints.size();
  ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", success,
           path_length, num_vertices);

  if (!success) {
    return false;
  }

  visualization_msgs::MarkerArray marker_array;
  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        waypoints, mav_visualization::Color::Green(), "rrt_star", 0.075));
    marker_array.markers.push_back(
        createMarkerForWaypoints(waypoints, mav_visualization::Color::Green(),
                                 "rrt_star_waypoints", 0.15));
  }

  last_waypoints_ = waypoints;

  if (!do_smoothing_) {
    last_trajectory_valid_ = true;
  } else {
    mav_msgs::EigenTrajectoryPointVector poly_path;
    mav_trajectory_generation::timing::Timer poly_timer("plan/poly");
    generateFeasibleTrajectory(waypoints, &poly_path);
    poly_timer.Stop();

    // Check all the paths.
    bool poly_has_collisions = checkPathForCollisions(poly_path, NULL);
    ROS_INFO("Smoothed path has collisions? %d", poly_has_collisions);

    if (!poly_has_collisions) {
      last_trajectory_valid_ = true;
    }

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          poly_path, mav_visualization::Color::Orange(), "poly", 0.075));
    }
  }

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  response.success = success;

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
  ROS_INFO_STREAM("Finished planning with start point: "
                  << start_pose.position_W.transpose()
                  << " and goal point: " << goal_pose.position_W.transpose());
  return success;
}

visualization_msgs::Marker RrtPlannerVoxblox::createMarkerForPath(
    mav_msgs::EigenTrajectoryPointVector& path,
    const std_msgs::ColorRGBA& color, const std::string& name,
    double scale) const {
  visualization_msgs::Marker path_marker;

  const int kMaxSamples = 1000;
  const int num_samples = path.size();
  int subsample = 1;
  while (num_samples / subsample > kMaxSamples) {
    subsample *= 10;
  }
  const double kMaxMagnitude = 1.0e4;

  path_marker.header.frame_id = frame_id_;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size() / subsample);
  int i = 0;
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    i++;
    if (i % subsample != 0) {
      continue;
    }
    // Check that we're in some reasonable bounds.
    // Makes rviz stop crashing.
    if (point.position_W.maxCoeff() > kMaxMagnitude ||
        point.position_W.minCoeff() < -kMaxMagnitude) {
      continue;
    }

    geometry_msgs::Point point_msg;
    tf::pointKindrToMsg(point.position_W, &point_msg);
    path_marker.points.push_back(point_msg);
  }

  return path_marker;
}

visualization_msgs::Marker RrtPlannerVoxblox::createMarkerForWaypoints(
    mav_msgs::EigenTrajectoryPointVector& path,
    const std_msgs::ColorRGBA& color, const std::string& name,
    double scale) const {
  visualization_msgs::Marker path_marker;

  path_marker.header.frame_id = frame_id_;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size());
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    geometry_msgs::Point point_msg;
    tf::pointKindrToMsg(point.position_W, &point_msg);
    path_marker.points.push_back(point_msg);
  }

  return path_marker;
}

bool RrtPlannerVoxblox::generateFeasibleTrajectory(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool RrtPlannerVoxblox::checkPathForCollisions(
    const mav_msgs::EigenTrajectoryPointVector& path, double* t) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      if (t != NULL) {
        *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
      }
      return true;
    }
  }
  return false;
}

double RrtPlannerVoxblox::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

bool RrtPlannerVoxblox::checkPhysicalConstraints(
    const mav_trajectory_generation::Trajectory& trajectory) {
  // Check min/max manually.
  // Evaluate min/max extrema

  std::vector<int> dimensions = {0, 1, 2};  // Evaluate dimensions in x, y and z
  mav_trajectory_generation::Extremum v_min, v_max, a_min, a_max;
  trajectory.computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::VELOCITY, dimensions, &v_min,
      &v_max);
  trajectory.computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::ACCELERATION, dimensions,
      &a_min, &a_max);

  ROS_INFO("V min/max: %f/%f, A min/max: %f/%f", v_min.value, v_max.value,
           a_min.value, a_max.value);

  // Create input constraints.
  // TODO(helenol): just store these as members...
  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.addConstraint(
      ICT::kFMin, (mav_msgs::kGravity -
                   constraints_.a_max));  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kFMax, (mav_msgs::kGravity +
                   constraints_.a_max));  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kVMax, constraints_.v_max);  // maximum velocity in [m/s].

  // Create feasibility object of choice (FeasibilityAnalytic,
  // FeasibilitySampling, FeasibilityRecursive).
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check(
      input_constraints);
  feasibility_check.settings_.setMinSectionTimeS(0.01);

  mav_trajectory_generation::InputFeasibilityResult feasibility =
      feasibility_check.checkInputFeasibilityTrajectory(trajectory);
  if (feasibility !=
      mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
    ROS_ERROR_STREAM(
        "Trajectory is input infeasible: "
        << mav_trajectory_generation::getInputFeasibilityResultName(
            feasibility));
    return false;
  }
  return true;
}

}  // namespace mav_planning
