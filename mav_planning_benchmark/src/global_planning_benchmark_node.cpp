#include "mav_planning_benchmark/global_planning_benchmark.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planning_benchmark");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::GlobalPlanningBenchmark node(nh, nh_private);
  ROS_INFO("Initialized global planning benchmark node.");

  ros::spin();
  return 0;
}
