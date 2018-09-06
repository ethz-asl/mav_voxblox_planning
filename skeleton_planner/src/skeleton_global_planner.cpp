#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>

#include "skeleton_planner/skeleton_global_planner.h"

namespace mav_planning {

SkeletonGlobalPlanner::SkeletonGlobalPlanner(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("map"),
      visualize_(true),
      voxblox_server_(nh_, nh_private_),
      skeleton_generator_() {
  constraints_.setParametersFromRos(nh_private_);

  std::string voxblox_path;
  nh_private_.param("voxblox_path", voxblox_path, voxblox_path);
  nh_private_.param("sparse_graph_path", sparse_graph_path_,
                    sparse_graph_path_);
  nh_private_.param("visualize", visualize_, visualize_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &SkeletonGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &SkeletonGlobalPlanner::publishPathCallback, this);

  // Load a file from the params.
  if (voxblox_path.empty()) {
    ROS_ERROR("Couldn't load map, empty filename.");
    return;
  }

  std::shared_ptr<voxblox::EsdfMap> esdf_map = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map);

  if (!voxblox_server_.loadMap(voxblox_path)) {
    ROS_ERROR("Coudldn't load ESDF map!");
  }

  ROS_INFO(
      "Size: %f VPS: %zu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // Also make a new skeleton layer and load it.
  // Make this as an unmanaged raw pointer, since we'll give it to skeleton
  // generator to own.
  voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer =
      new voxblox::Layer<voxblox::SkeletonVoxel>(
          voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
          voxblox_server_.getEsdfMapPtr()
              ->getEsdfLayerPtr()
              ->voxels_per_side());

  if (!voxblox::io::LoadBlocksFromFile<voxblox::SkeletonVoxel>(
          voxblox_path,
          voxblox::Layer<
              voxblox::SkeletonVoxel>::BlockMergingStrategy::kReplace,
          true, skeleton_layer)) {
    ROS_ERROR("Coudln't load skeleton layer.");
    return;
  }

  // Now set up the skeleton generator.
  skeleton_generator_.setEsdfLayer(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_generator_.setSkeletonLayer(skeleton_layer);
  skeleton_generator_.setMinGvdDistance(constraints_.robot_radius);
  skeleton_generator_.setGenerateByLayerNeighbors(true);

  // Set up the A* planners.
  skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayer());
  skeleton_planner_.setEsdfLayer(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
  }
}

void SkeletonGlobalPlanner::generateSparseGraph() {
  ROS_INFO("About to generate skeleton graph.");
  skeleton_generator_.updateSkeletonFromLayer();
  ROS_INFO("Re-populated from layer.");

  if (!sparse_graph_path_.empty() &&
      skeleton_generator_.loadSparseGraphFromFile(sparse_graph_path_)) {
    ROS_INFO_STREAM("Loaded sparse graph from file: " << sparse_graph_path_);
  } else {
    skeleton_generator_.generateSparseGraph();
    ROS_INFO("Generated skeleton graph.");
  }
  if (visualize_) {
    voxblox::Pointcloud pointcloud;
    std::vector<float> distances;
    skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(
        &pointcloud, &distances);

    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    voxblox::pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    skeleton_pub_.publish(ptcloud_pcl);

    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    voxblox::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    sparse_graph_pub_.publish(marker_array);
  }

  // Set up the graph planner.
  sparse_graph_planner_.setGraph(&skeleton_generator_.getSparseGraph());
  mav_trajectory_generation::timing::Timer kd_tree_init("plan/graph/setup");
  sparse_graph_planner_.setup();
  kd_tree_init.Stop();

  ROS_INFO_STREAM("Generation timings: " << std::endl
                                         << voxblox::timing::Timing::Print());
}

bool SkeletonGlobalPlanner::plannerServiceCallback(
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

  voxblox::Point start_point =
      start_pose.position_W.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point =
      goal_pose.position_W.cast<voxblox::FloatingPoint>();

  visualization_msgs::MarkerArray marker_array;

  bool run_astar_esdf = false;
  bool run_astar_diagram = true;
  bool run_astar_graph = true;

  if (run_astar_esdf) {
    // First, run just the ESDF A*...
    voxblox::AlignedVector<voxblox::Point> esdf_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_esdf_timer(
        "plan/astar_esdf");
    bool success = skeleton_planner_.getPathInEsdf(start_point, goal_point,
                                                   &esdf_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector esdf_path;
    convertCoordinatePathToPath(esdf_coordinate_path, &esdf_path);
    double path_length = computePathLength(esdf_path);
    int num_vertices = esdf_path.size();
    astar_esdf_timer.Stop();
    ROS_INFO("ESDF A* Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          esdf_path, frame_id_, mav_visualization::Color::Yellow(),
          "astar_esdf", 0.1));
    }
  }

  if (run_astar_graph) {
    voxblox::AlignedVector<voxblox::Point> graph_coordinate_path;
    mav_trajectory_generation::timing::Timer graph_timer("plan/graph");
    bool success = sparse_graph_planner_.getPath(start_point, goal_point,
                                                 &graph_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector graph_path;
    convertCoordinatePathToPath(graph_coordinate_path, &graph_path);
    double path_length = computePathLength(graph_path);
    int num_vertices = graph_path.size();
    graph_timer.Stop();
    ROS_INFO("Graph Planning Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          graph_path, frame_id_, mav_visualization::Color::Teal(), "graph_plan",
          0.25));
    }
  }

  if (run_astar_diagram) {
    voxblox::AlignedVector<voxblox::Point> diagram_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_diag_timer(
        "plan/astar_diag");
    bool success = skeleton_planner_.getPathUsingEsdfAndDiagram(
        start_point, goal_point, &diagram_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector diagram_path;
    convertCoordinatePathToPath(diagram_coordinate_path, &diagram_path);
    double path_length = computePathLength(diagram_path);
    int num_vertices = diagram_path.size();
    astar_diag_timer.Stop();
    ROS_INFO("Diag A* Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          diagram_path, frame_id_, mav_visualization::Color::Orange(),
          "astar_diag", 0.35));
    }
  }

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
}

void SkeletonGlobalPlanner::convertCoordinatePathToPath(
    const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) const {
  CHECK_NOTNULL(path);
  path->reserve(coordinate_path.size());

  for (const voxblox::Point& voxblox_point : coordinate_path) {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    path->push_back(point);
  }
}

double SkeletonGlobalPlanner::getMapDistance(
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

bool SkeletonGlobalPlanner::publishPathCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  /* if (!last_trajectory_valid_) {
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
  } */
  return true;
}

}  // namespace mav_planning
