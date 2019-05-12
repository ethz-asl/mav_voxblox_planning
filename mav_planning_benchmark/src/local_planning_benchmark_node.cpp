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

  int num_trials = 100;
  std::string results_path;
  bool exit_at_end = false;
  nh_private.param("results_path", results_path, results_path);
  nh_private.param("num_trials", num_trials, num_trials);
  nh_private.param("exit_at_end", exit_at_end, exit_at_end);

  const double min_density = 0.05;
  const double max_density = 0.50;
  const double density_increment = 0.05;

  // Derived...
  int num_densities = static_cast<int>(std::round((max_density - min_density) /
                                                  density_increment)) +
                      1;

  int trials_per_density = num_trials / num_densities;
  ROS_INFO_STREAM("[Local Planning Benchmark]: Trials per density: "
                  << trials_per_density << " num densities: " << num_densities);
  int trial_number = 0;

  for (int i = 0; i < num_densities; ++i) {
    if (!ros::ok()) {
      break;
    }
    double density = min_density + i * density_increment;
    for (int j = 0; j < trials_per_density; ++j) {
      if (!ros::ok()) {
        break;
      }
      srand(trial_number);
      node.generateWorld(density);
      srand(trial_number);
      node.runBenchmark(trial_number);
      trial_number++;
    }
  }

  if (!results_path.empty()) {
    node.outputResults(results_path);
  }

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << voxblox::timing::Timing::Print() << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());

  if (exit_at_end) {
    ros::shutdown();
  } else {
    ros::spin();
  }
  return 0;
}
