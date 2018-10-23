#include "mav_planning_benchmark/local_planning_benchmark.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planning_benchmark");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::LocalPlanningBenchmark node(nh, nh_private);
  ROS_INFO("Initialized local planning benchmark node.");

  int num_trials = 1;
  std::string results_path;
  nh_private.param("results_path", results_path, results_path);
  nh_private.param("num_trials", num_trials, num_trials);

  double density = 0.1;
  for (int i = 0; i < num_trials; i++) {
    node.generateWorld(density);
    node.runBenchmark(i);
  }

  if (!results_path.empty()) {
    node.outputResults(results_path);
  }

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());

  ros::spin();
  return 0;
}
