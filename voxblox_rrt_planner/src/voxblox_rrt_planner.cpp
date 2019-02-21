#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_rrt_planner/voxblox_rrt_planner.h"

namespace mav_planning {

VoxbloxRrtPlanner::VoxbloxRrtPlanner(const ros::NodeHandle& nh,
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
      rrt_(nh_, nh_private_),
      enable_loco_(true),
      enable_loco_2_(true),
      enable_poly_(true),
      marker_font_size_(1.f),
      status_position_x_(0.f),
      status_position_y_(0.f),
      status_position_z_(0.f) {
  constraints_.setParametersFromRos(nh_private_);

  std::string input_filepath;
  nh_private_.param("voxblox_path", input_filepath, input_filepath);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("robot_radius_margin", robot_radius_margin_,
                    robot_radius_margin_);

  nh_private_.param("num_repetitions", num_repetitions_, num_repetitions_);
  CHECK_GT(num_repetitions_, 0);

  nh_private_.param("enable_loco", enable_loco_, enable_loco_);
  nh_private_.param("enable_loco_2", enable_loco_2_, enable_loco_2_);
  nh_private_.param("enable_poly", enable_poly_, enable_poly_);

  nh_private_.param("marker_font_size", marker_font_size_, marker_font_size_);
  nh_private_.param("status_position_x", status_position_x_,
                    status_position_x_);
  nh_private_.param("status_position_y", status_position_y_,
                    status_position_y_);
  nh_private_.param("status_position_z", status_position_z_,
                    status_position_z_);

  ROS_WARN_STREAM("num_repetitions: " << num_repetitions_);
  ROS_WARN_STREAM("robot_radius: " << constraints_.robot_radius);
  ROS_WARN_STREAM("robot_radius_margin: " << robot_radius_margin_);
  ROS_WARN_STREAM("enable_loco: " << enable_loco_);
  ROS_WARN_STREAM("enable_loco_2: " << enable_loco_2_);
  ROS_WARN_STREAM("enable_poly: " << enable_poly_);

  ROS_WARN_STREAM("status_position_x: " << status_position_x_);
  ROS_WARN_STREAM("status_position_y: " << status_position_y_);
  ROS_WARN_STREAM("status_position_z: " << status_position_z_);

  std::string gt_esdf_file;
  nh_private_.param("esdf_ground_truth_map", gt_esdf_file, gt_esdf_file);
  if (!gt_esdf_file.empty()) {
    voxblox::Layer<voxblox::EsdfVoxel>::Ptr layer;
    voxblox::io::LoadLayer<voxblox::EsdfVoxel>(
        gt_esdf_file, true /*multi layer support on*/, &layer);
    esdf_ground_truth_map_.reset(new voxblox::EsdfMap(layer));
  }

  nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  polynomial_trajectory_pub_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
          "polynomial_trajectory", 1);

  status_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "planner_status", 1, true);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &VoxbloxRrtPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &VoxbloxRrtPlanner::publishPathCallback, this);

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
  double voxel_size =
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  ROS_INFO(
      "Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // TODO(helenol): figure out what to do with optimistic/pessimistic here.
  rrt_.setRobotRadius(constraints_.robot_radius * (1.0 + robot_radius_margin_));
  rrt_.setOptimistic(false);

  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  // Set up the path smoother as well.
  smoother_.setParametersFromRos(nh_private_);
  smoother_.setMinCollisionCheckResolution(voxel_size);
  smoother_.setMapDistanceCallback(std::bind(&VoxbloxRrtPlanner::getMapDistance,
                                             this, std::placeholders::_1));

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setMapDistanceCallback(std::bind(
      &VoxbloxRrtPlanner::getMapDistance, this, std::placeholders::_1));

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }
}

bool VoxbloxRrtPlanner::publishPathCallback(std_srvs::EmptyRequest& request,
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

void VoxbloxRrtPlanner::computeMapBounds(Eigen::Vector3d* lower_bound,
                                         Eigen::Vector3d* upper_bound) const {
  if (esdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

bool VoxbloxRrtPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  // Setup latest copy of map.
  if (!(esdf_map_ &&
        esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) &&
      !(tsdf_map_ &&
        tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0)) {
    ROS_ERROR("Both maps are empty!");
    return false;
  }
  // Figure out map bounds!
  computeMapBounds(&lower_bound_, &upper_bound_);

  ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                 << upper_bound_.transpose() << " size: "
                                 << (upper_bound_ - lower_bound_).transpose());

  // Inflate the bounds a bit.
  constexpr double kBoundInflationMeters = 0.5;
  // Don't in flate in z. ;)
  rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0),
                 upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0));
  rrt_.setupProblem();

  ROS_INFO("Planning path.");

  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    return false;
  }
  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    return false;
  }

  PlanningResult result;

  bool rrt_success = false;
  for (size_t repetition_idx = 0u;
       repetition_idx < static_cast<size_t>(num_repetitions_);
       ++repetition_idx) {
    result.paths_planned++;

    mav_msgs::EigenTrajectoryPoint::Vector waypoints;
    mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

    rrt_success =
        rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
    rrtstar_timer.Stop();
    double path_length = computePathLength(waypoints);
    int num_vertices = waypoints.size();
    ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", rrt_success,
             path_length, num_vertices);

    visualization_msgs::MarkerArray marker_array;
    if (visualize_) {
      if (rrt_success) {
        marker_array.markers.push_back(createMarkerForPath(
            waypoints, frame_id_, mav_visualization::Color::Green(), "rrt_star",
            0.075));
        marker_array.markers.push_back(createMarkerForWaypoints(
            waypoints, frame_id_, mav_visualization::Color::Green(),
            "rrt_star_waypoints", 0.15));
      } else {
        mav_msgs::EigenTrajectoryPoint::Vector empty_waypoints;
        marker_array.markers.push_back(createMarkerForPath(
            empty_waypoints, frame_id_, mav_visualization::Color::Green(),
            "rrt_star", 0.075));
        marker_array.markers.push_back(createMarkerForWaypoints(
            empty_waypoints, frame_id_, mav_visualization::Color::Green(),
            "rrt_star_waypoints", 0.15));

        // Push empty markers to erase old path.
        mav_msgs::EigenTrajectoryPointVector empty_path;
        marker_array.markers.push_back(createMarkerForPath(
            empty_path, frame_id_, mav_visualization::Color::Pink(), "loco",
            0.075));
        // Push empty markers to erase old path.
        marker_array.markers.push_back(createMarkerForPath(
            empty_path, frame_id_, mav_visualization::Color::Teal(), "loco_2",
            0.075));
        // Push empty markers to erase old path.
        marker_array.markers.push_back(createMarkerForPath(
            empty_path, frame_id_, mav_visualization::Color::Orange(), "poly",
            0.075));
      }
    }

    if (!rrt_success) {
      visualization_msgs::Marker status_marker =
          createPlanningStatusMarker(repetition_idx, false /*rrt success*/,
                                     false, true, false, true, false, true);
      status_pub_.publish(status_marker);
      continue;
    }

    last_waypoints_ = waypoints;

    if (!do_smoothing_) {
      last_trajectory_valid_ = true;
    } else {
      ROS_ERROR_STREAM("Ground truth available: "
                       << static_cast<bool>(esdf_ground_truth_map_));

      bool poly_path_is_in_collision_with_gt = true;
      bool poly_has_collisions = true;
      if (enable_poly_) {
        mav_msgs::EigenTrajectoryPointVector poly_path;
        mav_trajectory_generation::timing::Timer poly_timer("plan/poly");
        poly_has_collisions = !generateFeasibleTrajectory(
            waypoints, &poly_path, &poly_path_is_in_collision_with_gt);
        poly_timer.Stop();

        const double poly_path_length_m = computePathLength(poly_path);

        result.poly_self_collisions += poly_has_collisions ? 1u : 0u;
        result.poly_gt_collisions +=
            poly_path_is_in_collision_with_gt ? 1u : 0u;

        if (!(poly_path_is_in_collision_with_gt || poly_has_collisions)) {
          result.average_successful_poly_path_length_m =
              ((result.average_successful_poly_path_length_m *
                static_cast<double>(result.poly_success)) +
               poly_path_length_m) /
              static_cast<double>(result.poly_success + 1.0);
          result.poly_success++;
        }

        ROS_ERROR_STREAM(
            "POLY - path length: "
            << poly_path_length_m << "m | is in collision with ground truth: "
            << (poly_path_is_in_collision_with_gt && esdf_ground_truth_map_)
            << " | is in collision with own map: " << poly_has_collisions);

        if (!poly_has_collisions) {
          last_trajectory_valid_ = true;
        }

        if (visualize_) {
          // add node if in collision
          if (!poly_has_collisions && !poly_path_is_in_collision_with_gt) {
            marker_array.markers.push_back(createMarkerForPath(
                poly_path, frame_id_, mav_visualization::Color::Orange(),
                "poly", 0.075));
          } else {
            // Push empty markers to erase old path.
            mav_msgs::EigenTrajectoryPointVector empty_path;
            marker_array.markers.push_back(createMarkerForPath(
                empty_path, frame_id_, mav_visualization::Color::Orange(),
                "poly", 0.075));
          }
        }
      }
      bool loco_path_is_in_collision_with_gt = true;
      bool loco_has_collisions = true;
      if (enable_loco_) {
        mav_msgs::EigenTrajectoryPointVector loco_path;
        mav_trajectory_generation::timing::Timer loco_timer("plan/loco");
        loco_has_collisions = !generateFeasibleTrajectoryLoco(
            waypoints, &loco_path, &loco_path_is_in_collision_with_gt);
        loco_timer.Stop();

        const double loco_path_length_m = computePathLength(loco_path);

        result.loco_self_collisions += loco_has_collisions ? 1u : 0u;
        result.loco_gt_collisions +=
            loco_path_is_in_collision_with_gt ? 1u : 0u;

        result.average_successful_poly_path_length_m += 0.0;

        if (!(loco_path_is_in_collision_with_gt || loco_has_collisions)) {
          result.average_successful_loco_path_length_m =
              ((result.average_successful_loco_path_length_m *
                static_cast<double>(result.loco_success)) +
               loco_path_length_m) /
              static_cast<double>(result.loco_success + 1.0);
          result.loco_success++;
        }

        ROS_ERROR_STREAM(
            "LOCO - path length: "
            << loco_path_length_m << "m | is in collision with ground truth: "
            << (loco_path_is_in_collision_with_gt && esdf_ground_truth_map_)
            << " | is in collision with own map: " << loco_has_collisions);

        if (visualize_) {
          // optmization based on esdf, no soft constraints, resampling
          if (!loco_has_collisions && !loco_path_is_in_collision_with_gt) {
            marker_array.markers.push_back(createMarkerForPath(
                loco_path, frame_id_, mav_visualization::Color::Pink(), "loco",
                0.075));
          } else {
            // Push empty markers to erase old path.
            mav_msgs::EigenTrajectoryPointVector empty_path;
            marker_array.markers.push_back(createMarkerForPath(
                empty_path, frame_id_, mav_visualization::Color::Pink(), "loco",
                0.075));
          }
        }
      }

      bool loco_2_path_is_in_collision_with_gt = true;
      bool loco_2_has_collisions = true;
      if (enable_loco_2_) {
        mav_msgs::EigenTrajectoryPointVector loco_2_path;
        mav_trajectory_generation::timing::Timer loco_2_timer("plan/loco_2");
        loco_2_has_collisions = !generateFeasibleTrajectoryLoco2(
            waypoints, &loco_2_path, &loco_2_path_is_in_collision_with_gt);
        loco_2_timer.Stop();

        const double loco_2_path_length_m = computePathLength(loco_2_path);

        result.loco_2_self_collisions += loco_2_has_collisions ? 1u : 0u;
        result.loco_2_gt_collisions +=
            loco_2_path_is_in_collision_with_gt ? 1u : 0u;

        result.average_successful_poly_path_length_m += 0.0;

        if (!(loco_2_path_is_in_collision_with_gt || loco_2_has_collisions)) {
          result.average_successful_loco_2_path_length_m =
              ((result.average_successful_loco_2_path_length_m *
                static_cast<double>(result.loco_2_success)) +
               loco_2_path_length_m) /
              static_cast<double>(result.loco_2_success + 1.0);
          result.loco_2_success++;
        }

        if (visualize_) {
          // optmization based on esdf, soft constraints, resampling
          if (!loco_2_has_collisions) {
            marker_array.markers.push_back(createMarkerForPath(
                loco_2_path, frame_id_, mav_visualization::Color::Teal(),
                "loco_2", 0.075));
          } else {
            // Push empty markers to erase old path.
            mav_msgs::EigenTrajectoryPointVector empty_path;
            marker_array.markers.push_back(createMarkerForPath(
                empty_path, frame_id_, mav_visualization::Color::Teal(),
                "loco_2", 0.075));
          }
        }
      }

      visualization_msgs::Marker status_marker = createPlanningStatusMarker(
          repetition_idx, true /*rrt success*/, !loco_has_collisions,
          loco_path_is_in_collision_with_gt, !loco_2_has_collisions,
          loco_2_path_is_in_collision_with_gt, !poly_has_collisions,
          poly_path_is_in_collision_with_gt);
      status_pub_.publish(status_marker);
    }

    if (visualize_) {
      path_marker_pub_.publish(marker_array);
    }

    response.success = rrt_success;
  }

  ROS_WARN_STREAM(result.print());

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
  ROS_INFO_STREAM("Finished planning with start point: "
                  << start_pose.position_W.transpose()
                  << " and goal point: " << goal_pose.position_W.transpose());
  return rrt_success;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectory(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path,
    bool* path_is_in_collision_with_gt) {
  smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_is_in_collision_with_gt != nullptr && esdf_ground_truth_map_) {
    *path_is_in_collision_with_gt =
        checkPathForCollisionsWithGroundTruth(*path, NULL);
  }

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectoryLoco(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path,
    bool* path_is_in_collision_with_gt) {
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_is_in_collision_with_gt != nullptr) {
    *path_is_in_collision_with_gt =
        checkPathForCollisionsWithGroundTruth(*path, NULL);
  }

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectoryLoco2(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path,
    bool* path_is_in_collision_with_gt) {
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_is_in_collision_with_gt != nullptr) {
    *path_is_in_collision_with_gt =
        checkPathForCollisionsWithGroundTruth(*path, NULL);
  }

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::checkPathForCollisions(
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

bool VoxbloxRrtPlanner::checkPathForCollisionsWithGroundTruth(
    const mav_msgs::EigenTrajectoryPointVector& path, double* t) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistanceGroundTruth(point.position_W) <
        (constraints_.robot_radius * (1.0 - robot_radius_margin_))) {
      if (t != NULL) {
        *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
      }
      return true;
    }
  }
  return false;
}

double VoxbloxRrtPlanner::getMapDistanceGroundTruth(
    const Eigen::Vector3d& position) const {
  if (!esdf_ground_truth_map_) {
    LOG(FATAL) << "FUUUUCKED!";
    return 0.0;
  }
  double distance = 0.0;
  if (!esdf_ground_truth_map_->getDistanceAtPosition(position, &distance)) {
    return 0.0;
  }
  return distance;
}

double VoxbloxRrtPlanner::getMapDistance(
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

bool VoxbloxRrtPlanner::checkPhysicalConstraints(
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

visualization_msgs::Marker VoxbloxRrtPlanner::createPlanningStatusMarker(
    const size_t plan_number, const bool rrt_success, const bool loco_success,
    const bool loco_gt_violation, const bool loco_2_success,
    const bool loco_2_gt_violation, const bool poly_success,
    const bool poly_gt_violation) const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "planning_status";
  marker.id = 123;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = status_position_x_;
  marker.pose.position.y = status_position_y_;
  marker.pose.position.z = status_position_z_;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = marker_font_size_;
  marker.scale.y = marker_font_size_;
  marker.scale.z = marker_font_size_;

  marker.color.r = 1.0f;
  marker.color.g = rrt_success ? 1.0f : 0.0f;
  marker.color.b = rrt_success ? 1.0f : 0.0f;
  marker.color.a = 1.0f;

  std::stringstream ss;
  ss << "Plan                  " << plan_number << std::endl << std::endl;
  ss << "green  - RRT*           " << (rrt_success ? "success" : "failed")
     << std::endl;

  if (enable_loco_ && rrt_success) {
    ss << "pink   - ESDF planner     "
       << ((loco_success && !loco_gt_violation)
               ? "success"
               : ((loco_success && loco_gt_violation) ? "GT collision"
                                                      : "failed"))
       << std::endl;
  } else {
    ss << std::endl;
  }
  if (enable_poly_ && rrt_success) {
    ss << "orange - Polynomial planner  "
       << ((poly_success && !poly_gt_violation)
               ? "success"
               : ((poly_success && poly_gt_violation) ? "GT collision"
                                                      : "failed"))
       << std::endl;
  } else {
    ss << std::endl;
  }
  if (enable_loco_2_ && rrt_success) {
    ss << "teal    - ESDF planner V2  "
       << ((loco_2_success && !loco_2_gt_violation)
               ? "success"
               : ((loco_2_success && loco_2_gt_violation) ? "GT collision"
                                                          : "failed"))
       << std::endl;
  } else {
    ss << std::endl;
  }
  ss << "                                                                    ";

  marker.text = ss.str();

  return marker;
}

}  // namespace mav_planning
