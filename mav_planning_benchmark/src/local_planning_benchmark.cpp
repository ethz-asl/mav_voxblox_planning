#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_visualization/helpers.h>
#include <voxblox/core/common.h>
#include <voxblox/utils/planning_utils.h>

#include "mav_planning_benchmark/local_planning_benchmark.h"

namespace mav_planning {

LocalPlanningBenchmark::LocalPlanningBenchmark(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      frame_id_("map"),
      replan_dt_(1.0),
      max_replans_(60),
      lower_bound_(0.0, 0.0, 0.0),
      upper_bound_(15.0, 15.0, 5.0),
      camera_resolution_(320, 240),
      camera_fov_h_rad_(1.5708),  // 90 deg
      camera_min_dist_(0.5),
      camera_max_dist_(10.0),
      camera_model_dist_(5.0),
      loco_planner_(nh_, nh_private_),
      esdf_server_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);
  goal_selector_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  nh_private_.param("camera_max_dist", camera_max_dist_, camera_max_dist_);
  nh_private_.param("camera_model_dist", camera_model_dist_,
                    camera_model_dist_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  view_ptcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
      "view_ptcloud_pub", 1, true);
  additional_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path_additions",
                                                             1, true);

  esdf_server_.setClearSphere(true);

  loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());
  goal_selector_.setTsdfMap(esdf_server_.getTsdfMapPtr());
}

void LocalPlanningBenchmark::generateWorld(double density) {
  // There's a 2 meter padding on each side of the map that's free.
  const double kWorldXY = 15.0;
  const double kWorldZ = 5.0;

  generateCustomWorld(Eigen::Vector3d(kWorldXY, kWorldXY, kWorldZ), density);
}

void LocalPlanningBenchmark::runBenchmark(int trial_number) {
  constexpr double kPlanningHeight = 1.5;
  constexpr double kMinDistanceToGoal = 0.2;

  srand(trial_number);
  esdf_server_.clear();
  LocalBenchmarkResult result_template;

  result_template.trial_number = trial_number;
  result_template.seed = trial_number;
  result_template.density = density_;
  result_template.robot_radius_m = constraints_.robot_radius;
  result_template.v_max = constraints_.v_max;
  result_template.a_max = constraints_.a_max;

  mav_msgs::EigenTrajectoryPoint start, goal;
  start.position_W =
      Eigen::Vector3d(1.0, upper_bound_.y() / 2.0, kPlanningHeight);
  goal.position_W = upper_bound_ - start.position_W;
  goal.position_W.z() = kPlanningHeight;

  start.setFromYaw(0.0);
  goal.setFromYaw(0.0);
  result_template.straight_line_path_length_m =
      (goal.position_W - start.position_W).norm();

  visualization_msgs::MarkerArray marker_array, additional_markers;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory last_trajectory;
  mav_msgs::EigenTrajectoryPointVector executed_path;

  // Clear the visualization markers, if visualizing.
  if (visualize_) {
    visualization_msgs::Marker marker;
    for (int i = 0; i < max_replans_; ++i) {
      // Clear all existing stuff.
      marker.ns = "loco";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker_array.markers.push_back(marker);
    }
    marker.ns = "executed_path";
    marker.id = 0;
    marker_array.markers.push_back(marker);
    path_marker_pub_.publish(marker_array);
    marker_array.markers.clear();
    ros::spinOnce();
  }

  // In case we're finding new goal if needed, we have to keep track of which
  // goal we're currently tracking.
  mav_msgs::EigenTrajectoryPoint current_goal = goal;

  double start_time = 0.0;
  double plan_elapsed_time = 0.0;
  double total_path_distance = 0.0;
  int i = 0;
  for (i = 0; i < max_replans_; ++i) {
    if (i > 0 && !trajectory.empty()) {
      start_time = replan_dt_;
    }
    // Generate a viewpoint and add it to the map.
    mav_msgs::EigenTrajectoryPoint viewpoint;
    if (i == 0 || executed_path.empty()) {
      viewpoint = start;
    } else {
      viewpoint = executed_path.back();
    }
    addViewpointToMap(viewpoint);
    if (visualize_) {
      appendViewpointMarker(viewpoint, &additional_markers);
    }

    // Cache last real trajectory.
    last_trajectory = trajectory;

    // Actually plan the path.
    mav_trajectory_generation::timing::MiniTimer timer;
    bool success = false;

    if (i == 0) {
      success = loco_planner_.getTrajectoryTowardGoal(start, current_goal,
                                                      &trajectory);
    } else {
      success = loco_planner_.getTrajectoryTowardGoalFromInitialTrajectory(
          start_time, last_trajectory, current_goal, &trajectory);
    }
    plan_elapsed_time += timer.stop();

    if (!success || trajectory.empty()) {
      if (!goal_selector_.selectNextGoal(goal, current_goal, viewpoint,
                                         &current_goal)) {
        // In case we're not tracking a new goal...
        break;
      }
      continue;
    }

    // Sample the trajectory, set the yaw, and append to the executed path.
    mav_msgs::EigenTrajectoryPointVector path;
    if (!trajectory.empty()) {
      mav_trajectory_generation::sampleWholeTrajectory(
          trajectory, constraints_.sampling_dt, &path);
      setYawFromVelocity(start.getYaw(), &path);
    }

    // Append the next stretch of the trajectory. This will also take care of
    // the end.
    size_t max_index = std::min(
        static_cast<size_t>(std::floor(replan_dt_ / constraints_.sampling_dt)),
        path.size() - 1);
    executed_path.insert(executed_path.end(), path.begin(),
                         path.begin() + max_index);
    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          path, frame_id_,
          percentToRainbowColor(static_cast<double>(i) / max_replans_), "loco",
          0.075));
      marker_array.markers.back().id = i;
      path_marker_pub_.publish(marker_array);
      additional_marker_pub_.publish(additional_markers);
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }

    if ((executed_path.back().position_W - goal.position_W).norm() <
        kMinDistanceToGoal) {
      break;
    }
  }

  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        executed_path, frame_id_, mav_visualization::Color::Black(),
        "executed_path", 0.1));
    path_marker_pub_.publish(marker_array);
    additional_marker_pub_.publish(additional_markers);
    ros::spinOnce();
  }
  double path_length = computePathLength(executed_path);
  double distance_from_goal = (start.position_W - goal.position_W).norm();
  if (!executed_path.empty()) {
    distance_from_goal =
        (executed_path.back().position_W - goal.position_W).norm();
  }

  result_template.total_path_length_m = path_length;
  result_template.distance_from_goal = distance_from_goal;
  result_template.planning_success = distance_from_goal < kMinDistanceToGoal;
  result_template.num_replans = i;
  result_template.computation_time_sec = plan_elapsed_time;
  // Rough estimate. ;)
  result_template.total_path_time_sec =
      constraints_.sampling_dt * executed_path.size();
  result_template.is_collision_free = isPathCollisionFree(executed_path);
  result_template.is_feasible = isPathFeasible(executed_path);
  result_template.local_planning_method = kLoco;

  results_.push_back(result_template);
  ROS_INFO(
      "[Local Planning Benchmark] Trial number: %d Success: %d Replans: %d "
      "Final path length: %f Distance from goal: %f",
      trial_number, result_template.planning_success, i, path_length,
      distance_from_goal);
}

void LocalPlanningBenchmark::outputResults(const std::string& filename) {
  FILE* fp = fopen(filename.c_str(), "w+");
  if (fp == NULL) {
    return;
  }
  fprintf(fp,
          "#trial,seed,density,robot_radius,v_max,a_max,local_method,planning_"
          "success,is_collision_free,is_feasible,num_replans,distance_from_"
          "goal,computation_time_sec,total_path_time_sec,total_path_length_m,"
          "straight_line_path_length_m\n");
  for (const LocalBenchmarkResult& result : results_) {
    fprintf(fp, "%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f\n",
            result.trial_number, result.seed, result.density,
            result.robot_radius_m, result.v_max, result.a_max,
            result.local_planning_method, result.planning_success,
            result.is_collision_free, result.is_feasible, result.num_replans,
            result.distance_from_goal, result.computation_time_sec,
            result.total_path_time_sec, result.total_path_length_m,
            result.straight_line_path_length_m);
  }
  fclose(fp);
  ROS_INFO_STREAM("[Local Planning Benchmark] Output results to: " << filename);
}

void LocalPlanningBenchmark::generateCustomWorld(const Eigen::Vector3d& size,
                                                 double density) {
  esdf_server_.clear();

  lower_bound_ = Eigen::Vector3d::Zero();
  upper_bound_ = size;

  density_ = density;  // Cache this for result output.
  world_.clear();
  world_.addPlaneBoundaries(0.0, size.x(), 0.0, size.y());

  world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(
      voxblox::Point(0.0, 0.0, 0.0), voxblox::Point(0.0, 0.0, 1.0),
      voxblox::Color::Green())));

  // Sets the display bounds.
  world_.setBounds(
      Eigen::Vector3f(-1.0, -1.0, -1.0),
      size.cast<voxblox::FloatingPoint>() + Eigen::Vector3f(1.0, 1.0, 1.0));

  // Free space around the edges (so we know we don't start in collision).
  Eigen::Vector3d free_space_bounds(4.0, 4.0, 4.0);

  // Some mins and maxes... All objects gotta be on the floor. Just because.
  const double kMinHeight = 2.0;
  const double kMaxHeight = 5.0;
  const double kMinRadius = 0.25;
  const double kMaxRadius = 1.0;

  double usable_area = (size.x() - 2 * free_space_bounds.x()) *
                       (size.y() - 2 * free_space_bounds.y());
  int num_objects = static_cast<int>(std::floor(density * usable_area));

  for (int i = 0; i < num_objects; ++i) {
    // First select size; pose depends on size in z.
    double height = randMToN(kMinHeight, kMaxHeight);
    double radius = randMToN(kMinRadius, kMaxRadius);
    // First select its pose.
    Eigen::Vector3d position(
        randMToN(free_space_bounds.x(), size.x() - free_space_bounds.x()),
        randMToN(free_space_bounds.y(), size.y() - free_space_bounds.y()),
        height / 2.0);

    world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cylinder(
        position.cast<float>(), radius, height, voxblox::Color(165, 42, 42))));
  }

  esdf_server_.setSliceLevel(1.5);

  // Cache the TSDF voxel size.
  voxel_size_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxel_size();
}

// Generates a synthetic viewpoint, and adds it to the voxblox map.
void LocalPlanningBenchmark::addViewpointToMap(
    const mav_msgs::EigenTrajectoryPoint& viewpoint) {
  // Step 1: get the T_G_C for this viewpoint, and view origin + direction.
  // The yaw direction is now just coming from the trajectory, as the yaw
  // is assigned earlier for the whole trajectory chunk now.
  Eigen::Vector3f view_origin = viewpoint.position_W.cast<float>();
  Eigen::Vector3f view_direction(1.0, 0.0, 0.0);
  view_direction = viewpoint.orientation_W_B.cast<float>() * view_direction;

  // T_G_C is from z-positive since that's how camera coordinates work.
  voxblox::Transformation T_G_C(
      view_origin.cast<float>(),
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0.0, 0.0, 1.0),
                                         view_direction));
  // Step 2: actually get the pointcloud.
  voxblox::Pointcloud ptcloud, ptcloud_C;
  voxblox::Colors colors;
  world_.getPointcloudFromViewpoint(view_origin, view_direction,
                                    camera_resolution_, camera_fov_h_rad_,
                                    camera_max_dist_, &ptcloud, &colors);

  // Step 3: integrate into the map.
  // Transform back into camera frame.
  voxblox::transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);
  esdf_server_.integratePointcloud(T_G_C, ptcloud_C, colors);

  // Step 4: update mesh and ESDF. NewPoseCallback will mark unknown as
  // occupied and clear space otherwise.
  esdf_server_.newPoseCallback(T_G_C);
  if (visualize_) {
    esdf_server_.updateMesh();
  } else {
    esdf_server_.updateEsdf();
  }

  if (visualize_) {
    esdf_server_.publishAllUpdatedTsdfVoxels();
    esdf_server_.publishSlices();

    pcl::PointCloud<pcl::PointXYZRGB> ptcloud_pcl;
    ptcloud_pcl.header.frame_id = frame_id_;
    for (size_t i = 0; i < ptcloud.size(); ++i) {
      pcl::PointXYZRGB point;
      point.x = ptcloud[i].x();
      point.y = ptcloud[i].y();
      point.z = ptcloud[i].z();
      point.r = colors[i].r;
      point.g = colors[i].g;
      point.b = colors[i].b;
      ptcloud_pcl.push_back(point);
    }

    view_ptcloud_pub_.publish(ptcloud_pcl);
  }
}

double LocalPlanningBenchmark::getMapDistance(
    const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = true;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double LocalPlanningBenchmark::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = true;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

// Evaluate what we've got here.
bool LocalPlanningBenchmark::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      return false;
    }
  }
  return true;
}

bool LocalPlanningBenchmark::isPathFeasible(
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

void LocalPlanningBenchmark::appendViewpointMarker(
    const mav_msgs::EigenTrajectoryPoint& point,
    visualization_msgs::MarkerArray* marker_array) const {
  // First draw axes for these thing....
  visualization_msgs::Marker marker;
  constexpr double kAxesLength = 0.5;
  constexpr double kAxesWidth = 0.05;
  mav_visualization::drawAxes(point.position_W, point.orientation_W_B,
                              kAxesLength, kAxesWidth, &marker);
  marker.header.frame_id = frame_id_;
  marker.ns = "viewpoint_axes";

  marker_array->markers.push_back(marker);
}

void LocalPlanningBenchmark::setYawFromVelocity(
    double default_yaw, mav_msgs::EigenTrajectoryPointVector* path) {
  for (size_t i = 0; i < path->size(); ++i) {
    // Non-const ref that gets modified below.
    mav_msgs::EigenTrajectoryPoint& point = (*path)[i];

    if (point.velocity_W.norm() > 1e-6) {
      double yaw = atan2(point.velocity_W.y(), point.velocity_W.x());
      point.setFromYaw(yaw);
    } else {
      point.setFromYaw(default_yaw);
    }
  }
}

}  // namespace mav_planning
