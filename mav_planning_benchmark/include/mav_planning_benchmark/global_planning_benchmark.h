#ifndef MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_
#define MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_

#include <mav_path_smoothing/loco_smoother.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_path_smoothing/velocity_ramp_smoother.h>
#include <mav_planning_common/physical_constraints.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_rrt_planner/voxblox_ompl_rrt.h>
#include <voxblox_skeleton/io/skeleton_io.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/sparse_graph_planner.h>
#include <voxblox_skeleton_planner/skeleton_graph_planner.h>

namespace mav_planning {

class GlobalPlanningBenchmark {
 public:
  enum GlobalPlanningMethod {
    kStraightLine = 0,
    kRrtConnect,
    kRrtStar,
    kSkeletonGraph,
    kPrm,
    kBitStar
  };

  enum PathSmoothingMethod { kNone = 0, kVelocityRamp, kPolynomial, kLoco };
  struct GlobalBenchmarkResult {
    int trial_number = 0;
    int seed = 0;
    double robot_radius_m = 0.0;
    double v_max = 0.0;
    double a_max = 0.0;
    GlobalPlanningMethod global_planning_method;
    PathSmoothingMethod path_smoothing_method;
    bool planning_success = false;
    bool is_collision_free = false;
    bool is_feasible = false;
    double computation_time_sec = 0.0;
    double total_path_time_sec = 0.0;
    double total_path_length_m = 0.0;
    double straight_line_path_length_m = 0.0;
  };

  GlobalPlanningBenchmark(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);

  void loadMap(const std::string& base_path, const std::string& esdf_name,
               const std::string& sparse_graph_name);

  void runBenchmark(int num_trials);

  void outputResults(const std::string& filename);

 private:
  void setupPlanners();
  bool selectRandomStartAndGoal(double minimum_distance, Eigen::Vector3d* start,
                                Eigen::Vector3d* goal) const;

  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceWithoutInterpolation(
      const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;

  // Evaluate what we've got here.
  void fillInPathResults(const mav_msgs::EigenTrajectoryPointVector& path,
                         GlobalBenchmarkResult* result) const;
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

  // Functions to actually run the planners.
  bool runGlobalPlanner(const GlobalPlanningMethod planning_method,
                        const mav_msgs::EigenTrajectoryPoint& start,
                        const mav_msgs::EigenTrajectoryPoint& goal,
                        mav_msgs::EigenTrajectoryPointVector* waypoints);
  bool runPathSmoother(const PathSmoothingMethod smoothing_method,
                       const mav_msgs::EigenTrajectoryPointVector& waypoints,
                       mav_msgs::EigenTrajectoryPointVector* path);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS stuff.
  ros::Publisher path_marker_pub_;

  // Settings for physical constriants.
  PhysicalConstraints constraints_;

  // General settings.
  bool verbose_;
  bool visualize_;
  std::string frame_id_;

  // Voxblox Server!
  std::unique_ptr<voxblox::EsdfServer> esdf_server_;
  // Skeleton sparse graph!
  voxblox::SparseSkeletonGraph skeleton_graph_;

  // Map settings.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  // Global Planners!
  VoxbloxOmplRrt rrt_connect_planner_;
  VoxbloxOmplRrt rrt_star_planner_;
  VoxbloxOmplRrt bit_star_planner_;
  VoxbloxOmplRrt prm_planner_;
  SkeletonGraphPlanner skeleton_planner_;

  // Path Smoothers!
  VelocityRampSmoother ramp_smoother_;
  PolynomialSmoother poly_smoother_;
  LocoSmoother loco_smoother_;

  // Which methods to use.
  std::vector<GlobalPlanningMethod> global_planning_methods_;
  std::vector<PathSmoothingMethod> path_smoothing_methods_;

  // Results.
  std::vector<GlobalBenchmarkResult> results_;
};

}  // namespace mav_planning

#endif  // MAV_PLANNING_BENCHMKARK_GLOBAL_PLANNING_BENCHMARK_H_
