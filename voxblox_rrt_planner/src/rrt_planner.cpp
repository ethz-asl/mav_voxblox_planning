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
      last_trajectory_valid_(false) {
}

void RrtPlanner::getParametersFromRos() {
  constraints_.setParametersFromRos(nh_private_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);
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
  ROS_INFO("[RrtPlanner] advertized topics.");
}
void RrtPlanner::subscribeToTopics() {}

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

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  // Setup latest copy of map.
  if (!map_->isMapInitialized()) {
    return false;
  }
  // Figure out map bounds!
  Eigen::Vector3d lower_bound, upper_bound;
  map_->computeMapBounds(&lower_bound, &upper_bound);

  ROS_INFO_STREAM("Map bounds: " << lower_bound.transpose() << " to "
                                 << upper_bound.transpose() << " size: "
                                 << (upper_bound - lower_bound).transpose());
  setupRrtPlanner();

  ROS_INFO("Planning path.");

  if (map_->getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    return false;
  }
  if (map_->getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    return false;
  }

  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

  bool success = planRrt(start_pose, goal_pose, &waypoints);
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

}  // namespace mav_planning
