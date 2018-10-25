#ifndef MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
#define MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_

namespace mav_planning {

class MavLocalPlanner {
 public:
  MavLocalPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Input data.
  void waypointCallback(const geometry_msgs::PoseStamped& msg);
  void waypointListCallback(const geometry_msgs::PoseArray& msg);

  // Stops path publishing and clears all recent trajectories.
  void clearTrajectory();
  // Same as above, but also sends the current pose of the helicopter to the
  // controller to clear the queue.
  void abort();

  // Service callbacks.
  bool startCallback(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response);
  bool pauseCallback(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response);
  bool stopCallback(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response);

  // TODO -- TO IMPLEMENT:
  void polynomialTrajectoryCallback(
      const mav_planning_msgs::PolynomialTrajectory4D& msg) {}

 private:
  // Functions to help out replanning.





  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS inputs and outputs.
  ros::Subscriber waypoint_sub_;
  ros::Subscriber waypoint_list_sub_;
  // Current state of the MAV.
  ros::Subscriber odometry_sub_;

  ros::Publisher command_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher full_trajectory_pub_;

  // Service calls for controlling the local planner.
  // Start will start publishing commands, pause will stop temporarily and you
  // can call start to un-pause, and stop will stop and clear the current
  // trajectory.
  ros::ServiceServer start_srv_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer stop_srv_;

  // Service client for getting the MAV interface to listen to our sent
  // commands.
  ros::ServiceClient position_hold_client_;

  // ROS async handling: callback queues and spinners for command publishing,
  // which is on a separate thread from the rest of the functions.
  ros::CallbackQueue command_publishing_queue_;
  ros::AsyncSpinner command_publishing_spinner_;

  // Settings -- general
  bool verbose_;

  // Settings -- frames
  std::string global_frame_id_;
  std::string local_frame_id_;

  // Settings -- constraints.
  PhysicalConstraints constraints_;

  // Settings -- controller interface.
  int mpc_prediction_horizon_;  // Units: timesteps.
  double replan_dt_;

  // Settings -- general planning.
  bool use_obstacle_avoidance_;
  bool autostart_;  // Whether to auto-start publishing any new path or wait
  // for start service call.

  // State -- robot state.
  mav_msgs::EigenOdometry odometry_;

  // State -- waypoints.
  mav_msgs::EigenTrajectoryPointVector waypoints_;
  int current_waypoint_;

  // State -- current tracked path.
  mav_msgs::EigenTrajectoryPointVector path_;
  size_t path_index_;
  // Super important: mutex for locking the path queues.
  std::mutex path_mutex_;

  // Map!
  voxblox::EsdfServer esdf_server_;

  // Planners.

};

}  // namespace mav_planning

#endif  // MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_H_
