#include <mav_planning_common/utils.h>
#include <voxblox/utils/planning_utils.h>

#include "mav_planning_benchmark/global_planning_benchmark.h"

namespace mav_planning {

GlobalPlanningBenchmark::GlobalPlanningBenchmark(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      frame_id_("map"),
      rrt_planner_(nh, nh_private) {
  // Make sure we got the robot size, v_max, a_max, etc from constraints.
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
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

  skeleton_graph_.clear();
  if (!voxblox::io::loadSparseSkeletonGraphFromFile(sparse_graph_path,
                                                    &skeleton_graph_)) {
    ROS_ERROR_STREAM(
        "Couldn't load skeleton sparse graph from file: " << sparse_graph_path);
    return;
  }

  setupPlanners();
}

void GlobalPlanningBenchmark::setupPlanners() {
  CHECK(esdf_server_);
  esdf_server_->setTraversabilityRadius(constraints_.robot_radius);

  // For all planners:
  // Figure out map bounds!
  voxblox::utils::computeMapBoundsFromLayer(
      *esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(), &lower_bound_,
      &upper_bound_);
  double voxel_size =
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  // RRT*
  rrt_planner_.setRobotRadius(constraints_.robot_radius);
  rrt_planner_.setOptimistic(false);
  rrt_planner_.setEsdfLayer(esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  rrt_planner_.setBounds(lower_bound_, upper_bound_);
  rrt_planner_.setupProblem();

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
  skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);
  sparse_graph_planner_.setGraph(&skeleton_graph_);
  sparse_graph_planner_.setup();

  // Set up shortener.
  path_shortener_.setEsdfLayer(
      esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr());
  path_shortener_.setConstraints(constraints_);

  // Straight-line smoother.
  ramp_smoother_.setParametersFromRos(nh_private_);

  // Poly smoother.
  poly_smoother_.setParametersFromRos(nh_private_);
  poly_smoother_.setMinCollisionCheckResolution(voxel_size);
  poly_smoother_.setMapDistanceCallback(std::bind(
      &GlobalPlanningBenchmark::getMapDistance, this, std::placeholders::_1));

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setMapDistanceCallback(std::bind(
      &GlobalPlanningBenchmark::getMapDistance, this, std::placeholders::_1));
}

void GlobalPlanningBenchmark::runBenchmark(int num_trials) {
  CHECK(esdf_server_);

  const double kMinimumDistanceMeters = 2.0;

  for (int trial = 0; trial < num_trials; trial++) {
    srand(trial);
    // Get the start and goal positions.
    Eigen::Vector3d start, goal;
    if (!selectRandomStartAndGoal(kMinimumDistanceMeters, &start, &goal)) {
      ROS_ERROR(
          "Couldn't find a valid start and goal position at least %f meters "
          "apart! Aborting at trial %d.",
          kMinimumDistanceMeters, trial);
      return;
    }

    // Go through a list of all the global planners to try...

    // Go through a list of all the path smoothers to try, per planner.
  }
}

void GlobalPlanningBenchmark::outputResults(const std::string& filename) {}

bool GlobalPlanningBenchmark::selectRandomStartAndGoal(
    double minimum_distance, Eigen::Vector3d* start,
    Eigen::Vector3d* goal) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(goal);
  const int kMaxTries = 1000;
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
          getMapDistance(*goal) > constraints_.robot_radius) {
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
  if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                            &distance)) {
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
  // we
  // do the smoothing.
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max) {
      return false;
    }
  }
  return true;
}

}  // namespace mav_planning
