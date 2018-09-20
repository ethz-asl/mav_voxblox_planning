#ifndef MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_
#define MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_

namespace mav_planning {

struct GlobalBenchmarkResult {
  int trial_number = 0;
  int seed = 0;
  std::string map_name;
  double robot_radius_m = 0.0;
  int global_planning_method;
  int smoothing_method;
  bool has_collisions = false;
  double computation_time_sec = 0.0;
  double total_path_time_sec = 0.0;
  double total_path_length_m = 0.0;
  double straight_line_path_length_m = 0.0;
};

class GlobalPlanningBenchmark {
 public:








};

}  // namespace mav_planning

#endif  // MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_
