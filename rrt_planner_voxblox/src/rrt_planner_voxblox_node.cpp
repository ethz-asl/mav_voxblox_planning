#include "rrt_planner_voxblox/rrt_planner_voxblox.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rrt_planner_voxblox");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::RrtPlannerVoxblox node(nh, nh_private);
  ROS_INFO("Initialized RRT Planner Voxblox node.");

  ros::spin();
  return 0;
}
