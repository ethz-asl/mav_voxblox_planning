#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox/utils/planning_utils.h>

#include "mav_planning_benchmark/global_planning_benchmark.h"

namespace mav_planning {

GlobalPlanningBenchmark::GlobalPlanningBenchmark(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      frame_id_("map"),
      rrt_connect_planner_(nh, nh_private),
      rrt_star_planner_(nh, nh_private),
      bit_star_planner_(nh, nh_private),
      prm_planner_(nh, nh_private),
      skeleton_planner_(nh, nh_private) {
  // Make sure we got the robot size, v_max, a_max, etc from constraints.
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);

  global_planning_methods_.push_back(kStraightLine);
  global_planning_methods_.push_back(kRrtConnect);
  global_planning_methods_.push_back(kRrtStar);
  global_planning_methods_.push_back(kSkeletonGraph);
  global_planning_methods_.push_back(kPrm);

  path_smoothing_methods_.push_back(kNone);
  path_smoothing_methods_.push_back(kVelocityRamp);
  path_smoothing_methods_.push_back(kPolynomial);
  path_smoothing_methods_.push_back(kLoco);
}

void GlobalPlanningBenchmark::loadMap(const std::string& base_path,
                                      const std::string& esdf_name,
                                      const std::string& sparse_graph_name) {
  // We can add as many "/////"s as we want, let's play it safe.
  std::string esdf_path = base_path + "/" + esdf_name;
  std::string sparse_graph_path = base_path + "/" + sparse_graph_name;

  // TODO(helenol): replace this with actually loading the stats from the map.
  esdf_server_.reset(new voxblox::EsdfServer(nh_, nh_private_));

  if (!esdf_server_->loadMap(esdf_path)) {
    ROS_ERROR_STREAM("Couldn't load ESDF  from file: " << esdf_path);
    return;
  }
  esdf_server_->setTraversabilityRadius(constraints_.robot_radius);

  skeleton_graph_.clear();
  if (!voxblox::io::loadSparseSkeletonGraphFromFile(sparse_graph_path,
                                                    &skeleton_graph_)) {
    ROS_ERROR_STREAM(
        "Couldn't load skeleton sparse graph from file: " << sparse_graph_path);
    return;
  }

  if (visualize_) {
    esdf_server_->disableIncrementalUpdate();
    esdf_server_->updateMesh();
    esdf_server_->publishSlices();
    esdf_server_->publishPointclouds();
    esdf_server_->publishTraversable();
  }

  setupPlanners();
}

void GlobalPlanningBenchmark::setupPlanners() {
  CHECK(esdf_server_);

  // For all planners:
  // Figure out map bounds!
  voxblox::utils::computeMapBoundsFromLayer(
      *esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(), &lower_bound_,
      &upper_bound_);
  double voxel_size =
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  // RRT Connect
  rrt_connect_planner_.setPlanner(VoxbloxOmplRrt::kRrtConnect);
  rrt_connect_planner_.setNumSecondsToPlan(1.0);
  rrt_connect_planner_.setRobotRadius(constraints_.robot_radius);
  rrt_connect_planner_.setOptimistic(false);
  rrt_connect_planner_.setEsdfLayer(
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  rrt_connect_planner_.setBounds(lower_bound_, upper_bound_);
  rrt_connect_planner_.setupProblem();

  // RRT*
  rrt_star_planner_.setPlanner(VoxbloxOmplRrt::kRrtStar);
  rrt_star_planner_.setNumSecondsToPlan(2.0);
  rrt_star_planner_.setRobotRadius(constraints_.robot_radius);
  rrt_star_planner_.setOptimistic(false);
  rrt_star_planner_.setEsdfLayer(
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  rrt_star_planner_.setBounds(lower_bound_, upper_bound_);
  rrt_star_planner_.setupProblem();

  // BIT*
  bit_star_planner_.setPlanner(VoxbloxOmplRrt::kRrtConnect);
  bit_star_planner_.setNumSecondsToPlan(1.0);
  bit_star_planner_.setRobotRadius(constraints_.robot_radius);
  bit_star_planner_.setOptimistic(false);
  bit_star_planner_.setEsdfLayer(
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  bit_star_planner_.setBounds(lower_bound_, upper_bound_);
  bit_star_planner_.setupProblem();

  // PRM (note that this one is a bit different, since we actually
  prm_planner_.setPlanner(VoxbloxOmplRrt::kPrm);
  prm_planner_.setNumSecondsToPlan(0.01);
  prm_planner_.setRobotRadius(constraints_.robot_radius);
  prm_planner_.setOptimistic(false);
  prm_planner_.setEsdfLayer(esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  prm_planner_.setBounds(lower_bound_, upper_bound_);
  prm_planner_.setupProblem();
  // This is different from the other planners: grow the PRM tree for 2 seconds!
  prm_planner_.constructPrmRoadmap(2.0);

  //       .-.
  //      (o.o)
  //       |=|
  //      __|__
  //    //.=|=.\\
  //   // .=|=. \\
  //   \\ .=|=. //
  //    \\(_=_)//
  //     (:| |:)
  //      || ||
  //      () ()
  //      || ||
  //      || ||
  // l42 ==' '==
  skeleton_planner_.setEsdfLayer(
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_planner_.setRobotRadius(constraints_.robot_radius);
  skeleton_planner_.setSparseGraph(&skeleton_graph_);

  // Straight-line smoother.
  ramp_smoother_.setParametersFromRos(nh_private_);

  // Poly smoother.
  poly_smoother_.setParametersFromRos(nh_private_);
  poly_smoother_.setMinCollisionCheckResolution(voxel_size);
  poly_smoother_.setMapDistanceCallback(std::bind(
      &GlobalPlanningBenchmark::getMapDistance, this, std::placeholders::_1));
  poly_smoother_.setOptimizeTime(true);
  poly_smoother_.setSplitAtCollisions(true);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setDistanceAndGradientFunction(
      std::bind(&GlobalPlanningBenchmark::getMapDistanceAndGradient, this,
                std::placeholders::_1, std::placeholders::_2));
  loco_smoother_.setOptimizeTime(true);
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setResampleVisibility(true);
  loco_smoother_.setNumSegments(5);
}

void GlobalPlanningBenchmark::runBenchmark(int num_trials) {
  CHECK(esdf_server_);

  const double kMinimumDistanceMeters = 2.0;
  GlobalBenchmarkResult result_template;
  result_template.robot_radius_m = constraints_.robot_radius;
  result_template.v_max = constraints_.v_max;
  result_template.a_max = constraints_.a_max;

  for (int trial = 0; trial < num_trials; trial++) {
    srand(trial);
    if (!ros::ok()) {
      break;
    }

    // Get the start and goal positions.
    Eigen::Vector3d start, goal;
    if (!selectRandomStartAndGoal(kMinimumDistanceMeters, &start, &goal)) {
      ROS_ERROR(
          "Couldn't find a valid start and goal position at least %f meters "
          "apart! Aborting at trial %d.",
          kMinimumDistanceMeters, trial);
      return;
    }

    result_template.trial_number = trial;
    result_template.seed = trial;
    result_template.straight_line_path_length_m = (goal - start).norm();

    mav_msgs::EigenTrajectoryPoint start_point, goal_point;
    start_point.position_W = start;
    goal_point.position_W = goal;

    visualization_msgs::MarkerArray marker_array;

    // Go through a list of all the global planners to try...
    for (GlobalPlanningMethod global_method : global_planning_methods_) {
      srand(trial);

      mav_msgs::EigenTrajectoryPointVector waypoints;
      mav_trajectory_generation::timing::MiniTimer global_planner_timer;
      bool global_success =
          runGlobalPlanner(global_method, start_point, goal_point, &waypoints);
      global_planner_timer.stop();

      // Go through a list of all the path smoothers to try, per planner.
      for (PathSmoothingMethod smoothing_method : path_smoothing_methods_) {
        srand(trial);
        mav_msgs::EigenTrajectoryPointVector path;

        mav_trajectory_generation::timing::MiniTimer smoothing_timer;
        bool local_success =
            runPathSmoother(smoothing_method, waypoints, &path);
        smoothing_timer.stop();

        ROS_INFO(
            "[Trial %d]: Finished running global method %d and local method %d",
            trial, global_method, smoothing_method);

        GlobalBenchmarkResult result = result_template;
        result.global_planning_method = global_method;
        result.path_smoothing_method = smoothing_method;

        result.planning_success = global_success && local_success;
        result.computation_time_sec =
            global_planner_timer.getTime() + smoothing_timer.getTime();
        fillInPathResults(path, &result);
        results_.push_back(result);

        if (visualize_) {
          marker_array.markers.push_back(createMarkerForPath(
              path, frame_id_, percentToRainbowColor(global_method / 4.0 +
                                                     smoothing_method / 12.0),
              std::to_string(global_method) + "_" +
                  std::to_string(smoothing_method),
              0.075));
        }
      }
    }
    if (visualize_) {
      path_marker_pub_.publish(marker_array);
    }
  }
}

void GlobalPlanningBenchmark::outputResults(const std::string& filename) {
  // Append? That's cool I guess.
  FILE* fp = fopen(filename.c_str(), "w+");
  if (fp == NULL) {
    return;
  }
  fprintf(fp,
          "#trial,seed,robot_radius,v_max,a_max,global_method,smoothing_method,"
          "planning_success,is_collision_free,is_feasible,computation_time_sec,"
          "total_path_time_sec,total_path_length_m,straight_line_path_length_"
          "m\n");
  for (const GlobalBenchmarkResult& result : results_) {
    fprintf(fp, "%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f,%f\n",
            result.trial_number, result.seed, result.robot_radius_m,
            result.v_max, result.a_max, result.global_planning_method,
            result.path_smoothing_method, result.planning_success,
            result.is_collision_free, result.is_feasible,
            result.computation_time_sec, result.total_path_time_sec,
            result.total_path_length_m, result.straight_line_path_length_m);
  }
  fclose(fp);
  ROS_INFO_STREAM(
      "[Global Planning Benchmark] Output results to: " << filename);
}

bool GlobalPlanningBenchmark::selectRandomStartAndGoal(
    double minimum_distance, Eigen::Vector3d* start,
    Eigen::Vector3d* goal) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(goal);
  const int kMaxTries = 100000;
  bool solution_found = false;
  for (int i = 0; i < kMaxTries; i++) {
    *start << randMToN(lower_bound_.x(), upper_bound_.x()),
        randMToN(lower_bound_.y(), upper_bound_.y()),
        randMToN(lower_bound_.z(), upper_bound_.z());
    *goal << randMToN(lower_bound_.x(), upper_bound_.x()),
        randMToN(lower_bound_.y(), upper_bound_.y()),
        randMToN(lower_bound_.z(), upper_bound_.z());

    if ((*start - *goal).norm() > minimum_distance) {
      if (getMapDistance(*start) > constraints_.robot_radius &&
          getMapDistanceWithoutInterpolation(*start) >
              constraints_.robot_radius &&
          getMapDistance(*goal) > constraints_.robot_radius &&
          getMapDistanceWithoutInterpolation(*goal) >
              constraints_.robot_radius) {
        solution_found = true;
        break;
      }
    }
  }
  return solution_found;
}

double GlobalPlanningBenchmark::getMapDistance(
    const Eigen::Vector3d& position) const {
  CHECK(esdf_server_);
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double GlobalPlanningBenchmark::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  CHECK(esdf_server_);
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

double GlobalPlanningBenchmark::getMapDistanceWithoutInterpolation(
    const Eigen::Vector3d& position) const {
  CHECK(esdf_server_);
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

bool GlobalPlanningBenchmark::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      return false;
    }
  }
  return true;
}

bool GlobalPlanningBenchmark::isPathFeasible(
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

void GlobalPlanningBenchmark::fillInPathResults(
    const mav_msgs::EigenTrajectoryPointVector& path,
    GlobalBenchmarkResult* result) const {
  if (path.empty()) {
    return;
  }
  result->is_collision_free = isPathCollisionFree(path);
  result->is_feasible = isPathFeasible(path);
  result->total_path_time_sec =
      mav_msgs::nanosecondsToSeconds(path.back().time_from_start_ns);
  result->total_path_length_m = computePathLength(path);
}

bool GlobalPlanningBenchmark::runGlobalPlanner(
    const GlobalPlanningMethod planning_method,
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPointVector* waypoints) {
  CHECK_NOTNULL(waypoints);
  if (planning_method == kStraightLine) {
    waypoints->push_back(start);
    waypoints->push_back(goal);
    return true;
  }
  if (planning_method == kRrtConnect) {
    bool success =
        rrt_connect_planner_.getPathBetweenWaypoints(start, goal, waypoints);
    return success;
  }
  if (planning_method == kRrtStar) {
    bool success =
        rrt_star_planner_.getPathBetweenWaypoints(start, goal, waypoints);
    return success;
  }
  if (planning_method == kSkeletonGraph) {
    bool success =
        skeleton_planner_.getPathBetweenWaypoints(start, goal, waypoints);
    return success;
  }
  if (planning_method == kPrm) {
    bool success = prm_planner_.getPathBetweenWaypoints(start, goal, waypoints);
    return success;
  }

  return false;
}

bool GlobalPlanningBenchmark::runPathSmoother(
    const PathSmoothingMethod smoothing_method,
    const mav_msgs::EigenTrajectoryPointVector& waypoints,
    mav_msgs::EigenTrajectoryPointVector* path) {
  CHECK_NOTNULL(path);
  if (smoothing_method == kNone) {
    *path = waypoints;
    return true;
  }
  if (smoothing_method == kVelocityRamp) {
    bool success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    return success;
  }

  if (smoothing_method == kPolynomial) {
    bool success = poly_smoother_.getPathBetweenWaypoints(waypoints, path);
    return success;
  }

  if (smoothing_method == kLoco) {
    bool success = false;
    if (waypoints.size() == 2) {
      success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                       waypoints[1], path);
    } else {
      success = loco_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    return success;
  }
}

}  // namespace mav_planning
