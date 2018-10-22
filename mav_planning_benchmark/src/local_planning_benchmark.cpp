#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
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
      lower_bound_(2.5, 0.0, 0.0),
      upper_bound_(12.5, 10.0, 5.0),
      camera_resolution_(320, 240),
      camera_fov_h_rad_(1.5708),  // 90 deg
      camera_min_dist_(0.5),
      camera_max_dist_(10.0),
      camera_model_dist_(5.0),
      esdf_server_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);

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
}

void LocalPlanningBenchmark::generateWorld(double density) {
  const double kWorldXY = 10.0;
  const double kWorldZ = 5.0;

  generateCustomWorld(Eigen::Vector3d(kWorldXY, kWorldXY, kWorldZ), density);
}

void LocalPlanningBenchmark::runBenchmark(int trial_number) {}

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

  density_ = density;  // Cache this for result output.
  world_.clear();
  world_.addPlaneBoundaries(0.0, size.x(), 0.0, size.y());
  world_.addGroundLevel(0.0);
  // Sets the display bounds.
  world_.setBounds(
      Eigen::Vector3f(-1.0, -1.0, -1.0),
      size.cast<voxblox::FloatingPoint>() + Eigen::Vector3f(1.0, 1.0, 1.0));

  // Free space around the edges (so we know we don't start in collision).
  Eigen::Vector3d free_space_bounds(2.0, 2.0, 2.0);

  // Some mins and maxes... All objects gotta be on the floor. Just because.
  const double kMinHeight = 2.0;
  const double kMaxHeight = 5.0;
  const double kMinRadius = 0.25;
  const double kMaxRadius = 1.0;

  double usable_area = size.x() * size.y();
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
        position.cast<float>(), radius, height, voxblox::Color::Gray())));
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
  const bool kInterpolate = false;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double LocalPlanningBenchmark::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = false;
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

}  // namespace mav_planning
