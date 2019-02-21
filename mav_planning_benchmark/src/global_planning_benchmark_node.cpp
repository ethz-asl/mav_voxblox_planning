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

  int num_trials = 100;
  std::string base_path, esdf_name, sparse_graph_name, results_name;
  nh_private.param("base_path", base_path, base_path);
  nh_private.param("esdf_name", esdf_name, esdf_name);
  nh_private.param("sparse_graph_name", sparse_graph_name, sparse_graph_name);
  nh_private.param("results_name", results_name, results_name);
  nh_private.param("num_trials", num_trials, num_trials);

  node.loadMap(base_path, esdf_name, sparse_graph_name);
  node.runBenchmark(num_trials);
  node.outputResults(base_path + "/" + results_name);

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());

  ros::spin();
  return 0;
}
