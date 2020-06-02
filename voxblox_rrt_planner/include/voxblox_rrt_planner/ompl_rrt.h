#ifndef VOXBLOX_RRT_PLANNER_OMPL_RRT_H_
#define VOXBLOX_RRT_PLANNER_OMPL_RRT_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include "voxblox_rrt_planner/ompl/mav_setup.h"

namespace mav_planning {

class BloxOmplRrt {
 public:
  enum RrtPlannerType {
    kRrtConnect = 0,
    kRrtStar,
    kInformedRrtStar,
    kBitStar,
    kPrm
  };

  BloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~BloxOmplRrt() {}

  inline void setRobotRadius(double robot_radius) {
    robot_radius_ = robot_radius;
  }
  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound);

  inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }
  bool getOptimistic() const { return optimistic_; }

  double getNumSecondsToPlan() const { return num_seconds_to_plan_; }
  void setNumSecondsToPlan(double num_seconds) {
    num_seconds_to_plan_ = num_seconds;
  }

  RrtPlannerType getPlanner() const { return planner_type_; }
  void setPlanner(RrtPlannerType planner) { planner_type_ = planner; }

  // Only call this once, only call this after setting all settings correctly.
  virtual void setupProblem();

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric& path,
      mav_msgs::EigenTrajectoryPointVector* trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal,
                             mav_msgs::EigenTrajectoryPoint::Vector* solution);

  void constructPrmRoadmap(double roadmap_construction_sec) {
    problem_setup_->clear();
    problem_setup_->setup();
    problem_setup_->constructPrmRoadmap(roadmap_construction_sec);
  }

 protected:
  void setupFromStartAndGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal);

  double getDistanceEigenToState(const Eigen::Vector3d& eigen,
                                 const ompl::base::State* state_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Setup the problem in OMPL.
  ompl::mav::MavSetup* problem_setup_;
  RrtPlannerType planner_type_;
  double num_seconds_to_plan_;
  bool simplify_solution_;
  double robot_radius_;
  bool verbose_;

  // Whether the planner is optimistic (true) or pessimistic (false) about
  // how unknown space is handled.
  // Optimistic uses the TSDF for collision checking, while pessimistic uses
  // the ESDF. Be sure to set the maps accordingly.
  bool optimistic_;

  // Whether to trust an approximate solution (i.e., not necessarily reaching
  // the exact goal state).
  bool trust_approx_solution_;

  // Planning bounds, if set.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  double voxel_size_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_OMPL_RRT_H_