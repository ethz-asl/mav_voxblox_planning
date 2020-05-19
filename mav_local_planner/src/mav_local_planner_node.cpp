#include "mav_local_planner/mav_local_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_local_planner");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::MavLocalPlanner node(nh, nh_private);
  ROS_INFO("Initialized Mav Local Planner node.");

  ros::spin();
  return 0;
}
