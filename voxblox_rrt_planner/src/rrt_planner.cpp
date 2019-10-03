#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include <mav_msgs/default_topics.h>

#include "voxblox_rrt_planner/rrt_planner.h"
#include <mav_planning_msgs/PlannerServiceResponse.h>

#include <boost/format.hpp>
#include <istream>

namespace mav_planning {

RrtPlanner::RrtPlanner(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("odom"),
      visualize_(true),
      do_smoothing_(true),
      random_start_goal_(false),
      last_trajectory_valid_(false) {
  ROS_INFO("[RrtPlanner] initialized.");
}

void RrtPlanner::getParametersFromRos() {
  constraints_.setParametersFromRos(nh_private_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);
  nh_private_.param("random_start_goal", random_start_goal_, random_start_goal_);
}
void RrtPlanner::advertiseTopics() {
  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  polynomial_trajectory_pub_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
          "polynomial_trajectory", 1);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &RrtPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &RrtPlanner::publishPathCallback, this);

  manual_trigger_srv_ = nh_private_.advertiseService(
      "manual_action", &RrtPlanner::manualTriggerCallback, this);
  ROS_INFO("[RrtPlanner] advertized topics.");
}
void RrtPlanner::subscribeToTopics() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &RrtPlanner::odometryCallback, this);
}

bool RrtPlanner::publishPathCallback(std_srvs::EmptyRequest& request,
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

bool RrtPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;
  ROS_INFO("[RrtPlanner] planning request received");

  mav_trajectory_generation::timing::Timer rrttotal_timer("plan/rrt_total");
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  // Setup latest copy of map.
  if (!map_->isMapInitialized()) {
    return false;
  }

  // Figure out map bounds!
  Eigen::Vector3d lower_bound, upper_bound;
  map_->computeMapBounds(&lower_bound, &upper_bound);

  // darpa hack
//  lower_bound.z() = 0;
//  upper_bound.z() = 3;

  /*
  // ARCHE interior limits
  lower_bound.x() = -22;
  lower_bound.y() = -12;
  lower_bound.z() =   1;
  upper_bound.x() = -11;
  */

  ROS_INFO_STREAM("Map bounds: " << lower_bound.transpose() << " to "
                                 << upper_bound.transpose() << " size: "
                                 << (upper_bound - lower_bound).transpose());
  setupRrtPlanner();

  ROS_INFO("Planning path.");
  ROS_INFO_STREAM("(" << start_pose.position_W.transpose() << ") to ("
      << goal_pose.position_W.transpose() << ") - "
      << (start_pose.position_W - goal_pose.position_W).norm());

  bool random_start = false, random_goal = false;
  if (map_->getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    random_start = true;
  }
  if (map_->getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    random_goal = true;
  }

  if (random_start_goal_ and (goal_pose.position_W - start_pose.position_W).norm() < 2) {
    random_goal = true;
  }

  ROS_INFO("[RrtPlanner] set random: %d", random_start_goal_);
  if (!random_start_goal_ and (random_start or random_goal)) {
    ROS_WARN("[RrtPlanner] distances: start %.2f, goal %.2f",
        map_->getMapDistance(start_pose.position_W), map_->getMapDistance(goal_pose.position_W));
    ROS_WARN("[RrtPlanner] weights: start %.2f, goal %.2f",
             map_->getMapWeight(start_pose.position_W), map_->getMapWeight(goal_pose.position_W));
    response.success = false;
    return false;
  }
  if (random_start and random_start_goal_) {
    srand(time(nullptr));
    double distance = 0;
    int counter = 0;
    while (distance < constraints_.robot_radius and counter < 1000) {
      Eigen::Vector3d random = Eigen::Vector3d::Random();
      start_pose.position_W = random.cwiseProduct(
          (upper_bound - lower_bound) / 2) + ((upper_bound + lower_bound) / 2);
      start_pose.position_W << randMToN(lower_bound.x(), upper_bound.x()),
          randMToN(lower_bound.y(), upper_bound.y()),
          randMToN(lower_bound.z(), upper_bound.z());
      distance = map_->getMapDistance(start_pose.position_W);
      counter++;
    }
    ROS_WARN_STREAM("Start set to [" << start_pose.position_W.transpose() << "]");
  }
  if (random_goal and random_start_goal_) {
    srand(time(nullptr));
    double distance = 0;
    int counter = 0;
    goal_pose.position_W = start_pose.position_W;
    while (counter < 1000 and (distance < constraints_.robot_radius or
                               (goal_pose.position_W - start_pose.position_W).norm() < 2)) {
      goal_pose.position_W = Eigen::Vector3d::Random().cwiseProduct(
          (upper_bound - lower_bound) / 2) + ((upper_bound + lower_bound) / 2);
      goal_pose.position_W << randMToN(lower_bound.x(), upper_bound.x()),
          randMToN(lower_bound.y(), upper_bound.y()),
          randMToN(lower_bound.z(), upper_bound.z());
      distance = map_->getMapDistance(goal_pose.position_W);
      counter++;
    }
    ROS_WARN_STREAM("Goal set to [" << goal_pose.position_W.transpose() << "]");
    ROS_INFO_STREAM("path length "
                        << (goal_pose.position_W - start_pose.position_W).norm());
  }

  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

  bool success = planRrt(start_pose, goal_pose, &waypoints);
  rrtstar_timer.Stop();
  rrttotal_timer.Stop();
  double path_length = computePathLength(waypoints);
  int num_vertices = waypoints.size();
  ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", success,
           path_length, num_vertices);

  if (!success) {
    visualization_msgs::MarkerArray marker_array;
    if (visualize_) {
      waypoints.emplace_back(start_pose);
      waypoints.emplace_back(goal_pose);

      marker_array.markers.push_back(createMarkerForPath(
          waypoints, frame_id_, mav_visualization::Color::White(), "rrt_star",
          0.075));
      marker_array.markers.push_back(createMarkerForWaypoints(
          waypoints, frame_id_, mav_visualization::Color::White(),
          "rrt_star_waypoints", 0.15));
    }
    path_marker_pub_.publish(marker_array);

    ROS_INFO_STREAM("All timings: "
                    << std::endl
                    << mav_trajectory_generation::timing::Timing::Print());
    ROS_INFO_STREAM("Finished planning with start point: "
                    << start_pose.position_W.transpose()
                    << " and goal point: " << goal_pose.position_W.transpose());
    return false;
  }

  visualization_msgs::MarkerArray marker_array;
  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        waypoints, frame_id_, mav_visualization::Color::Green(), "rrt_star",
        0.075));
    marker_array.markers.push_back(createMarkerForWaypoints(
        waypoints, frame_id_, mav_visualization::Color::Green(),
        "rrt_star_waypoints", 0.15));
  }

  last_waypoints_ = waypoints;

  if (!do_smoothing_) {
    last_trajectory_valid_ = true;
  } else {
    mav_msgs::EigenTrajectoryPointVector poly_path;
    mav_trajectory_generation::timing::Timer poly_timer("plan/poly");
    bool poly_has_collisions =
        !generateFeasibleTrajectory(waypoints, &poly_path);
    poly_timer.Stop();

    mav_msgs::EigenTrajectoryPointVector loco_path;
    mav_trajectory_generation::timing::Timer loco_timer("plan/loco");
    bool loco_has_collisions =
        !generateFeasibleTrajectoryLoco(waypoints, &loco_path);
    loco_timer.Stop();

    mav_msgs::EigenTrajectoryPointVector loco2_path;
    mav_trajectory_generation::timing::Timer loco2_timer("plan/loco2");
    bool loco2_has_collisions =
        !generateFeasibleTrajectoryLoco2(waypoints, &loco2_path);
    loco2_timer.Stop();

    ROS_INFO(
        "Poly Smoothed Path has collisions? %d Loco Path has collisions? %d "
        "Loco 2 has collisions? %d",
        poly_has_collisions, loco_has_collisions, loco2_has_collisions);

    if (!poly_has_collisions) {
      last_waypoints_ = poly_path;
      last_trajectory_valid_ = true;
    }

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          poly_path, frame_id_, mav_visualization::Color::Orange(), "poly",
          0.075));
      marker_array.markers.push_back(
          createMarkerForPath(loco_path, frame_id_,
                              mav_visualization::Color::Pink(), "loco", 0.075));
      marker_array.markers.push_back(createMarkerForPath(
          loco2_path, frame_id_, mav_visualization::Color::Teal(), "loco2",
          0.075));
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

bool RrtPlanner::generateFeasibleTrajectory(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool RrtPlanner::generateFeasibleTrajectoryLoco(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  loco_smoother_.setResampleTrajectory(false);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool RrtPlanner::generateFeasibleTrajectoryLoco2(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool RrtPlanner::checkPathForCollisions(
    const mav_msgs::EigenTrajectoryPointVector& path, double* t) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (map_->getMapDistance(point.position_W) < constraints_.robot_radius) {
      if (t != NULL) {
        *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
      }
      return true;
    }
  }
  return false;
}

bool RrtPlanner::checkPhysicalConstraints(
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

void RrtPlanner::explore() {
  ROS_INFO("[RrtPlanner] flying trajectory");
  bool maze = false;
  bool darpa_map = false;
  bool loop_closure = true;

  bool fix_height = false;

  std::vector<Eigen::Vector3d> waypoints;
  Eigen::Vector3d point;
  std::string filename;
  if (loop_closure) {
    // darpa edgar loop closure
    filename = "/media/darpa/SSD_500GB/gasserl/datas/darpa_loop_closure.txt";
    fix_height = true;
    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94,210,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(119,217,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(145,181,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94,210,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(119,217,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(145,181,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94,210,1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(145,181,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(119,217,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94,210,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(145,181,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(119,217,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94,210,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(115,178,1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(239,141,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235,157,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(239,141,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235,157,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235,157,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(239,141,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235,157,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(239,141,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(192,122,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185,134,1);
    waypoints.emplace_back(point);

    waypoints.clear();

    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185, 134, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(238, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(253, 91, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235, 157, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185, 134, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(238, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(253, 91, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235, 157, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185, 134, 1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(235, 157, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(253, 91, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(238, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185, 134, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(235, 157, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(253, 91, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(238, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(185, 134, 1);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(189, 163, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);

    // darpa practice loop closure
    filename = "/media/darpa/SSD_500GB/gasserl/datas/darpa_loop_closure.txt";
    waypoints.clear();
    fix_height = false;

    point = Eigen::Vector3d(0, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(15, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(40, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(22, -37, 1.5);
    waypoints.emplace_back(point);
//    point = Eigen::Vector3d(60, -40, 1.5);
//    waypoints.emplace_back(point);
    point = Eigen::Vector3d(77, -42, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(80, -60, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(62, -57, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(56, -1, 1.5);
    waypoints.emplace_back(point);
  } else if (darpa_map) {
    filename = "/media/darpa/SSD_500GB/gasserl/datas/darpa_edgar_waypoints.txt";
    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(38, 69, 2);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(74, 47, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(78, 66, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(85, 53, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(124, 80, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(32, 73, 2); // perf
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(43, 139, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(66, 217, 2);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(145, 181, 2);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(104, 242, 2);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(94, 210, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(115, 178, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(188, 163, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(417, 253, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(233, 172, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(223, 126, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180, 108, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(188, 163, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(271, 134, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(286, 92, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(345, 111, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(369, 84, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(390, 102, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(380, 84, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(448, 228, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(448, 240, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(484, 240, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(484, 228, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(447, 193, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(467, 166, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(467, 166, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(440, 167, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(385, 69, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(357, 67, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(340, 54, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(298, 70, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(252, 68, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(258, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(238, 24, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(214, 1, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(152, 25, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(140, 80, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180, 108, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(204, 104, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(193, 58, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(258, 72, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(204, 104, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(164, 75, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(152, 25, 2); // gut
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(122, 100, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);
    ROS_INFO("[RrtPlanner] got waypoints");
  } else if (maze) {
    filename = "/media/darpa/SSD_500GB/gasserl/datas/big_maze_waypoints.txt";

    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.5, 0,1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(3, 4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-1, 4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.5, 6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-4.5, 6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(3, -2, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.75, -2, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.75, -4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-4.5, -5.75, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-3, -12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-4.5, -12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-3, -12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(5.5, -10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(5.5, -14, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(12.5, -16, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.5, -16, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-4.5, -16, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-14.5, -12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-8.5, -8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-15, 10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-12.5, 2, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-10.5, -14, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-6.5, -12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-10.5, -10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-3, -10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-10, -5.75, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-4.75, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-12.5, -5.5, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-12.5, 8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-8.5, 2, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-8.5, 4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-14.75, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-9, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-6.5, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-3, 10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-0.75, 6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(-2.5, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(1.5, 6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(5.25, 4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(7.5, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(9, 8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(11, 8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(9.5, -10, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(13.25, -8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(9.5, -8, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(9.5, -6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(3.5, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(3.5, 2, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(7.5, 0, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(7.5, 2.25, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(11, 2.25, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(11.25, -7.5, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(13, -6, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(13.5, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(11.5, 12, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(11.5, 4, 1);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(0, 0, 1);
    waypoints.emplace_back(point);
    ROS_INFO("[RrtPlanner] got waypoints");
  } else {
    filename = "/media/darpa/SSD_500GB/gasserl/datas/darpa_practice2.txt";

    point = Eigen::Vector3d(0, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(15, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(40, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(62,57, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(90,60,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(110,60,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(140,39,-3.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(224,99,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(250,100,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(278,97,-8.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(280,40,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(240,-5,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(280,40,-8.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(323,2,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(358,-17,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(300,-40,-8.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(337,-178,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-160,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,-160,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(100,-140,-2.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(80,-100,1.5);

    point = Eigen::Vector3d(90,-120,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(110,-120,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,-120,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-120,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(283, -142, -8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(300,-40,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(280,0,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(280,40,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(250,100,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(140,40,-3.5);
    waypoints.emplace_back(point);

    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,20,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,20,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180,-60,-7);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,20,-8.5);
    waypoints.emplace_back(point);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,20,-3.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(110,40,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(90,40,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(90,20,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(110,20,-3.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(150,-60,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-60,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180,-60,-11);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(190,-60,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(237,-82,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(260,-140,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(237,-82,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(190,-60,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180,-60,-11);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-60,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,-60,-3.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(150,-80,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-80,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(180,-80,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(170,-80,-8.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(150,-80,-3.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(110,-60,-3.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(90,-60,1.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(80,-60,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(76,-41,1.5);
    waypoints.emplace_back(point);
//    point = Eigen::Vector3d(60,-40,15);
//    waypoints.emplace_back(point);
    point = Eigen::Vector3d(20,-30,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(40, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(56,-2,1.5);
    waypoints.emplace_back(point);
//    point = Eigen::Vector3d(60,-40,15);
//    waypoints.emplace_back(point);
    point = Eigen::Vector3d(63,-57,1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(80, -60, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(80, -100, 1.5);
    waypoints.emplace_back(point);

    point = Eigen::Vector3d(15, 0, 1.5);
    waypoints.emplace_back(point);
    point = Eigen::Vector3d(0, 0, 1.5);
    waypoints.emplace_back(point);
  }

  if (visualize_ and true) {
    visualization_msgs::MarkerArray start_goal_msgs;
    visualization_msgs::Marker vertex_marker;
    vertex_marker.header.frame_id = frame_id_;
    vertex_marker.ns = "waypoints";
    vertex_marker.id = 0;
    vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_marker.pose.orientation.w = 1.0;
    vertex_marker.scale.x = constraints_.robot_radius*2;
    vertex_marker.scale.y = vertex_marker.scale.x;
    vertex_marker.scale.z = vertex_marker.scale.x;
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = frame_id_;
    edge_marker.ns = "waypoint_edges";
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 1;
    edge_marker.scale.y = edge_marker.scale.x;
    edge_marker.scale.z = edge_marker.scale.x;
    std_msgs::ColorRGBA color_msg;
    color_msg.r = 1.0;
    color_msg.g = 1.0;
    color_msg.b = 1.0;
    color_msg.a = 0.5;
    for (ulong i = 0; i < waypoints.size() - 1; i++) {
      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(waypoints[i], point_msg);
      vertex_marker.points.push_back(point_msg);
      edge_marker.points.push_back(point_msg);
      tf::pointEigenToMsg(waypoints[i + 1], point_msg);
      edge_marker.points.push_back(point_msg);

      color_msg.r = 1.0;
      color_msg.g = 1.0;
      color_msg.b = 1.0;
      edge_marker.colors.push_back(color_msg);
      edge_marker.colors.push_back(color_msg);
      if (map_->getMapDistance(waypoints[i]) > constraints_.robot_radius) {
        color_msg.r = 0.0;
        color_msg.g = 1.0;
        color_msg.b = 0.0;
      } else {
        map_->setVerbose(true);
        map_->getMapDistance(waypoints[i]);
        color_msg.r = 1.0;
        color_msg.g = 0.0;
        color_msg.b = 0.0;
        map_->setVerbose(false);
      }
      vertex_marker.colors.push_back(color_msg);
    }
    start_goal_msgs.markers.push_back(vertex_marker);
    start_goal_msgs.markers.push_back(edge_marker);
    path_marker_pub_.publish(start_goal_msgs);
  }

  mav_msgs::EigenTrajectoryPointVector all_waypoints;
  double height = 1.5;
  if (fix_height) {
    waypoints[0].z() = height;
  }
  for (ulong i = 0; i < waypoints.size() - 1; i++) {
    if (fix_height) {
      waypoints[i + 1].z() = height;
    }

    // getting plan
    mav_planning_msgs::PlannerServiceRequest plan_request;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = waypoints[i].x();
    pose.pose.position.y = waypoints[i].y();
    pose.pose.position.z = waypoints[i].z();
    plan_request.start_pose = pose;
    pose.pose.position.x = waypoints[i + 1].x();
    pose.pose.position.y = waypoints[i + 1].y();
    pose.pose.position.z = waypoints[i + 1].z();
    plan_request.goal_pose = pose;
    mav_planning_msgs::PlannerServiceResponse plan_response;
    ROS_INFO("[RrtPlanner] prep %lu", i);

    Eigen::Vector3d diff = waypoints[i+1] - waypoints[i];
    if (abs(diff.x()) == 20 and abs(diff.z()) == 5) {
      ROS_WARN("[RrtPlanner] Probably ramp, no planning");
    } else if (abs(diff.x()) == 10 and abs(diff.z()) == 2.5) {
      ROS_WARN("[RrtPlanner] Probably dip, no planning");
//    } else if ((waypoints[i].x() == 0 and waypoints[i].y() == 0) or
//        (waypoints[i+1].x() == 0 and waypoints[i+1].y() == 0)) {
//      ROS_WARN("[RrtPlanner] Start problem");
    } else {
      plannerServiceCallback(plan_request, plan_response);
//    ROS_INFO("[RrtPlanner] planned");
    }

    if (visualize_) {
      visualization_msgs::MarkerArray start_goal_msgs;

      visualization_msgs::Marker edge_marker;
      edge_marker.header.frame_id = frame_id_;
      edge_marker.ns = "waypoint_edges";
      edge_marker.id = i + 1;
      edge_marker.type = visualization_msgs::Marker::LINE_LIST;
      edge_marker.pose.orientation.w = 1.0;
      edge_marker.scale.x = 0.5;
      edge_marker.scale.y = edge_marker.scale.x;
      edge_marker.scale.z = edge_marker.scale.x;

      visualization_msgs::Marker vertex_marker;
      vertex_marker.header.frame_id = frame_id_;
      vertex_marker.ns = "waypoints";
      vertex_marker.id = i + 1;
      vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      vertex_marker.pose.orientation.w = 1.0;
      vertex_marker.scale.x = constraints_.robot_radius*2;
      vertex_marker.scale.y = vertex_marker.scale.x;
      vertex_marker.scale.z = vertex_marker.scale.x;

      std_msgs::ColorRGBA color_msg;
      color_msg.b = 0.0;
      color_msg.a = 1.0;

      if (plan_response.success) {
        color_msg.r = 0.0;
        color_msg.g = 1.0;
      } else {
        color_msg.r = 1.0;
        color_msg.g = 0.0;
      }
      edge_marker.colors.push_back(color_msg);
      edge_marker.colors.push_back(color_msg);

      if (map_->getMapDistance(waypoints[i + 1]) >= constraints_.robot_radius) {
        color_msg.r = 0.0;
        color_msg.g = 1.0;
      } else {
        color_msg.r = 1.0;
        color_msg.g = 0.0;
      }
      vertex_marker.colors.push_back(color_msg);

      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(waypoints[i], point_msg);
      edge_marker.points.push_back(point_msg);
      tf::pointEigenToMsg(waypoints[i + 1], point_msg);
      edge_marker.points.push_back(point_msg);
      vertex_marker.points.push_back(point_msg);

      start_goal_msgs.markers.push_back(edge_marker);
      start_goal_msgs.markers.push_back(vertex_marker);
      path_marker_pub_.publish(start_goal_msgs);
    }

    if (!plan_response.success) {
      int continuation = 0;
//      if (map_->getMapDistance(waypoints[i + 1]) >= constraints_.robot_radius) {
      continuation = 1;
//      }
      while (continuation != 1) {
        std::cout << "continue (1), abort (2), redo(3) : ";
        std::cin >> continuation;
        if (continuation == 2) {
          return;
        }
        if (continuation == 3) {
          plannerServiceCallback(plan_request, plan_response);
        }
      }
//      waypoints[i+1] = waypoints[i];
      mav_msgs::EigenTrajectoryPoint failed_point_msg;
      failed_point_msg.position_W = waypoints[i];
      all_waypoints.insert(all_waypoints.begin(), failed_point_msg);
      failed_point_msg.position_W = waypoints[i+1];
      all_waypoints.insert(all_waypoints.begin(), failed_point_msg);
    }
//    continue;
    if (plan_response.success) {
      all_waypoints.insert(all_waypoints.begin(), last_waypoints_.rbegin(), last_waypoints_.rend());
    }
  }
  std::reverse(all_waypoints.begin(), all_waypoints.end());
  if (fix_height) {
    for (mav_msgs::EigenTrajectoryPoint& waypoint : all_waypoints) {
      waypoint.position_W.z() = height;
    }
  }

  if (visualize_) {
    visualization_msgs::MarkerArray explore_path_markers;
    explore_path_markers.markers.push_back(createMarkerForPath(
        all_waypoints, frame_id_, mav_visualization::Color::White(), "explore_path",
        0.3));
    path_marker_pub_.publish(explore_path_markers);

    std::vector<int> collision_edges;
    for (ulong i = 0; i < all_waypoints.size() - 1; i++) {
      bool collision = map_->checkCollision(all_waypoints[i].position_W, all_waypoints[i+1].position_W,
          constraints_.robot_radius);
      if (!collision) {
//      ROS_INFO_STREAM("[RrtPlanner] " << i << " th segment collision free ["
//          << all_waypoints[i].position_W.transpose() << "] to [" << all_waypoints[i+1].position_W.transpose() << "]");
      } else {
        ROS_ERROR("[GlobalGraphPlanner] %ldth segment in collision "
                  "[%.2f %.2f %.2f] to [%.2f %.2f %.2f]",
            i, all_waypoints[i].position_W.x(), all_waypoints[i].position_W.y(),
            all_waypoints[i].position_W.z(), all_waypoints[i+1].position_W.x(),
            all_waypoints[i+1].position_W.y(), all_waypoints[i+1].position_W.z());
        collision_edges.emplace_back(i+1);
//      return;
        visualization_msgs::MarkerArray start_goal_msgs;

        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = frame_id_;
        edge_marker.ns = "collision";
        edge_marker.id = i + 1;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.5;
        edge_marker.scale.y = edge_marker.scale.x;
        edge_marker.scale.z = edge_marker.scale.x;

        std_msgs::ColorRGBA color_msg;
        color_msg.b = 0.0;
        color_msg.a = 1.0;
        color_msg.r = 1.0;
        color_msg.g = 0.0;
        edge_marker.colors.push_back(color_msg);
        edge_marker.colors.push_back(color_msg);

        geometry_msgs::Point point_msg;
        tf::pointEigenToMsg(all_waypoints[i].position_W, point_msg);
        edge_marker.points.push_back(point_msg);
        tf::pointEigenToMsg(all_waypoints[i+1].position_W, point_msg);
        edge_marker.points.push_back(point_msg);

        start_goal_msgs.markers.push_back(edge_marker);
        path_marker_pub_.publish(start_goal_msgs);
      }
    }
  }

  // save to file
  std::ofstream outfile;
  outfile.open(filename);
  for (const mav_msgs::EigenTrajectoryPoint &waypoint : all_waypoints) {
    outfile << boost::format("%.2f,%.2f,%.2f\n")
               % waypoint.position_W.x() % waypoint.position_W.y() % waypoint.position_W.z();
  }
  outfile.close();
}

void RrtPlanner::flyPath() {
  std::string filename = "/media/darpa/SSD_500GB/gasserl/datas/waypoints.txt";

  ROS_INFO_STREAM("reading file " << filename);
  std::string line;
  mav_msgs::EigenTrajectoryPointVector point_list_msg;
  std::ifstream infile(filename);
  while(std::getline(infile, line)) {
    std::istringstream ss(line);
    std::string value;
    int coord = 0;
    mav_msgs::EigenTrajectoryPoint point_msg;
    while(std::getline(ss, value, ',')) {
      point_msg.position_W[coord] = std::stod(value);
      coord++;
    }
    point_list_msg.emplace_back(point_msg);
  }
  infile.open(filename);
  if (!infile.is_open()) {
    ROS_ERROR("could not open file!");
    return;
  }
  infile.close();

  // invert plan
  int invert = 1;
  std::cout << "inverting path? yes (1) or no (0) ";
  std::cin >> invert;
  if (invert == 1) {
    std::reverse(point_list_msg.begin(), point_list_msg.end());
  }

  // check plan
  ROS_INFO("checking path for collision (%lu segments)", point_list_msg.size() - 1);

  std::vector<int> collision_edges;
  visualization_msgs::MarkerArray path_msgs;
  visualization_msgs::MarkerArray collision_msgs;
  for (ulong i = 0; i < point_list_msg.size() - 1; i++) {
    bool collision = map_->checkCollision(point_list_msg[i].position_W, point_list_msg[i+1].position_W,
        constraints_.robot_radius);

    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = frame_id_;
    edge_marker.id = i + 1;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.5;
    edge_marker.scale.y = edge_marker.scale.x;
    edge_marker.scale.z = edge_marker.scale.x;
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(point_list_msg[i].position_W, point_msg);
    edge_marker.points.push_back(point_msg);
    tf::pointEigenToMsg(point_list_msg[i+1].position_W, point_msg);
    edge_marker.points.push_back(point_msg);
    std_msgs::ColorRGBA color_msg;

    if (!collision) {
//      ROS_INFO_STREAM("[RrtPlanner] " << i << " th segment collision free ["
//          << point_list_msg[i].position_W.transpose() << "] to [" << point_list_msg[i+1].position_W.transpose() << "]");
    } else {
      ROS_ERROR("[GlobalGraphPlanner] %ldth segment in collision "
                "[%.2f %.2f %.2f] to [%.2f %.2f %.2f]",
          i, point_list_msg[i].position_W.x(), point_list_msg[i].position_W.y(),
          point_list_msg[i].position_W.z(), point_list_msg[i+1].position_W.x(),
          point_list_msg[i+1].position_W.y(), point_list_msg[i+1].position_W.z());
//      return;

      visualization_msgs::Marker collision_marker = edge_marker;
      collision_marker.ns = "collision";
      color_msg.b = 0.0;
      color_msg.a = 1.0;
      color_msg.r = 1.0;
      color_msg.g = 0.0;
      collision_marker.colors.push_back(color_msg);
      collision_marker.colors.push_back(color_msg);
      collision_msgs.markers.push_back(collision_marker);
    }

    visualization_msgs::Marker path_marker = edge_marker;
    path_marker.ns = "path";
    color_msg.b = 1.0;
    color_msg.a = 1.0;
    color_msg.r = 1.0;
    color_msg.g = 1.0;
    path_marker.colors.push_back(color_msg);
    path_marker.colors.push_back(color_msg);
    path_msgs.markers.push_back(path_marker);
  }
  path_marker_pub_.publish(path_msgs);
  path_marker_pub_.publish(collision_msgs);

  // publishing plan
  last_waypoints_ = point_list_msg;
  last_trajectory_valid_ = true;

  std_srvs::EmptyRequest publish_request;
  std_srvs::EmptyResponse publish_response;
  publishPathCallback(publish_request, publish_response);
  ROS_INFO("[RrtPlanner] published\n");
}

void RrtPlanner::odometryCallback(const nav_msgs::Odometry &msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
}

bool RrtPlanner::manualTriggerCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  ROS_INFO("Manual action started.");

  int action = 0;
  std::cout << "explore (1) or fly (2)? ";
  std::cin >> action;
  if (action == 1) {
    explore();
  } else if (action == 2) {
    flyPath();
  }

  ROS_INFO("Manual action finished.");
  return true;
}

}  // namespace mav_planning