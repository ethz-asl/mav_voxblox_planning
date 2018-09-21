#include "mav_planning_benchmark/global_planning_benchmark.h"

namespace mav_planning {

GlobalPlanningBenchmark::GlobalPlanningBenchmark(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

void GlobalPlanningBenchmark::loadMap(const std::string& base_path,
                                      const std::string& esdf_name,
                                      const std::string& sparse_graph_name);

void GlobalPlanningBenchmark::runBenchmark(int num_trials);

void GlobalPlanningBenchmark::outputResults(const std::string& filename);

}  // namespace mav_planning
