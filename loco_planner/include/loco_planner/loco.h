#ifndef LOCO_PLANNER_LOCO_H_
#define LOCO_PLANNER_LOCO_H_

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <functional>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/timing.h>

namespace loco_planner {

// Templated on the same as the polynomial optimization.
// N coefficients, describing a polynomial of order N-1.
template <int N = 10>
class Loco {
  static_assert(N % 2 == 0, "The number of coefficients has to be even.");

 public:
  struct Config {
    int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::JERK;
    double epsilon = 0.5;
    double robot_radius = 0.5;
    bool soft_goal_constraint = false;
    double w_d = 0.1;   // Smoothness cost weight.
    double w_c = 10.0;  // Collision cost weight.
    double w_g = 2.5;   // Soft goal cost weight (if using soft goals).
    double w_w = 1.0;   // Waypoint cost weight (if waypoints set).
    double min_collision_sampling_dt = 0.1;
    double map_resolution = 0.1;  // Size of voxels in the map.
    bool verbose = false;
  };

  typedef std::function<double(const Eigen::VectorXd& position)>
      DistanceFunctionType;
  typedef std::function<double(const Eigen::VectorXd& position,
                               Eigen::VectorXd* gradient)>
      DistanceAndGradientFunctionType;

  Loco(size_t dimension);
  Loco(size_t dimension, const Config& config);

  // Setup the problem from positions or trajectory points.
  void setupFromPositions(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal, size_t num_segments,
                          double total_time);
  void setupFromTrajectoryPoints(
      const mav_msgs::EigenTrajectoryPoint& start_point,
      const mav_msgs::EigenTrajectoryPoint& goal_point, size_t num_segments,
      double total_time);
  void setupFromVertices(double total_time,
                         mav_trajectory_generation::Vertex::Vector* vertices);

  // These methods take in an initial solution:
  // The first method keeps the number of segments of the original, while the
  // second re-samples it into num_segments.
  // set_waypoints will determine whether initial vertices are set as soft costs
  // in the optimization.
  void setupFromTrajectory(
      const mav_trajectory_generation::Trajectory& trajectory);
  void setupFromTrajectoryAndResample(
      const mav_trajectory_generation::Trajectory& trajectory,
      size_t num_segments);

  // Optionally set soft costs on waypoints, at specific times:
  void setWaypoints(const std::map<double, Eigen::VectorXd>& waypoints);
  void setWaypointsFromTrajectory(
      const mav_trajectory_generation::Trajectory& trajectory);

  // Set how to get the distance of a point. ONE OF THESE TWO *MUST* BE SET!
  void setDistanceFunction(const DistanceFunctionType& function) {
    distance_function_ = function;
    distance_and_gradient_function_ =
        std::bind(&Loco::getNumericalDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2);
  }
  void setDistanceAndGradientFunction(
      const DistanceAndGradientFunctionType& function) {
    distance_and_gradient_function_ = function;
  }

  // This should probably return something...
  void solveProblem();

  // Different solver methods.
  void solveProblemCeres();
  void solveProblemGradientDescent();

  // Accessors for getting the data back out...
  void getTrajectory(mav_trajectory_generation::Trajectory* trajectory) const;

  // The final cost in the optimization.
  double getCost() const;

  // Output [t, p_x, p_y, ...] for MATLAB plotting.
  void printMatlabSampledTrajectory(double dt) const;

  // Updates the free parameters of the underlying optimization.
  void setFreeDerivatives(const std::vector<Eigen::VectorXd>& d_p);
  void getFreeDerivatives(std::vector<Eigen::VectorXd>* d_p) const;
  void setParameterVector(const Eigen::VectorXd& parameters);
  void getParameterVector(Eigen::VectorXd* parameters) const;
  int getK() const { return K_; }
  int getNumParams() const { return num_free_; }

  // Set parameters. Defaults are okay, but these should be modifiable.
  double getEpsilon() const { return config_.epsilon; }
  void setEpsilon(double epsilon) { config_.epsilon = epsilon; }
  double getRobotRadius() const { return config_.robot_radius; }
  void setRobotRadius(double robot_radius) {
    config_.robot_radius = robot_radius;
  }
  double getWd() const { return config_.w_d; }
  void setWd(double w_d) { config_.w_d = w_d; }
  double getWc() const { return config_.w_c; }
  void setWc(double w_c) { config_.w_c = w_c; }
  double getWg() const { return config_.w_g; }
  void setWg(double w_g) { config_.w_g = w_g; }
  double getWw() const { return config_.w_w; }
  void setWw(double w_w) { config_.w_w = w_w; }
  double getCollisionSamplingDt() const {
    return config_.min_collision_sampling_dt;
  }
  void setCollisionSamplingDt(double dt) {
    config_.min_collision_sampling_dt = dt;
  }
  double getMapResolution() const { return config_.map_resolution; }
  void setMapResolution(double map_resolution) {
    config_.map_resolution = map_resolution;
  }
  bool getSoftGoalConstraint() const { return config_.soft_goal_constraint; }
  void setSoftGoalConstraint(bool soft_goal) {
    config_.soft_goal_constraint = soft_goal;
  }
  bool getVerbose() const { return config_.verbose; }
  void setVerbose(bool verbose) { config_.verbose = verbose; }

  // Internal functions, for testing, debugging, or advanced use.

  // Returns cost and gradient vector, with respect to k (separate for each
  // dimension).
  double computeTotalCostAndGradients(
      std::vector<Eigen::VectorXd>* gradients) const;
  // Just the J_d part.
  double computeDerivativeCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients) const;
  // Just the J_c part.
  double computeCollisionCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients) const;
  // The J_g part if using soft goals.
  double computeGoalCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients) const;
  // The J_w part if the waypoint soft constraint list isn't empty.
  double computeWaypointCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients) const;

  // Actual function to compute specific costs per waypoint, also used to
  // calculate J_g above.
  double computePositionSoftCostAndGradient(
      double t, const Eigen::VectorXd& position,
      std::vector<Eigen::VectorXd>* gradients) const;

  double computePotentialCostAndGradient(const Eigen::VectorXd& position,
                                         Eigen::VectorXd* gradient) const;
  double potentialFunction(double distance) const;
  void potentialGradientFunction(double distance,
                                 const Eigen::VectorXd& distance_gradient,
                                 Eigen::VectorXd* gradient_out) const;
  // Convenience.
  void getTVector(double t, Eigen::VectorXd* T) const;

 private:
  void setupProblem();

  double getNumericalDistanceAndGradient(const Eigen::VectorXd& position,
                                         Eigen::VectorXd* gradient);

  // Private class for ceres evaluations.
  class NestedCeresFunction : public ceres::FirstOrderFunction {
   public:
    NestedCeresFunction(int K, int num_free, Loco* parent)
        : K_(K), num_free_(num_free), parent_(parent) {}

    virtual bool Evaluate(const double* parameters, double* cost,
                          double* gradient) const;
    virtual int NumParameters() const;

   private:
    int K_;
    int num_free_;
    Loco* parent_;
  };

  // This is where you store the matrices.
  mav_trajectory_generation::PolynomialOptimization<N> poly_opt_;

  // Collision cost functions.
  DistanceFunctionType distance_function_;
  DistanceAndGradientFunctionType distance_and_gradient_function_;

  // Most of the configuration settings for the optimization.
  Config config_;

  // If soft goal constraint is set, then save the goal location.
  Eigen::VectorXd goal_pos_;
  // If this map isn't empty, then waypoint costs are activated.
  // Maps from time (in trajectory time) to waypoint positions.
  std::map<double, Eigen::VectorXd> waypoints_;

  // Cache these params.
  int K_;  // Dimension of the problem.
  int num_free_;
  int num_fixed_;

  // Cache of all the relevant matrices.
  // These only depend on the segment time and number of free params.
  // Removes some flexibility (can't have different constraints in each axis)
  // but should speed up computation.
  Eigen::MatrixXd R_;
  Eigen::MatrixXd A_inv_;
  Eigen::MatrixXd M_;
  Eigen::MatrixXd V_;
  // L = A_inv * M (short-hand, frequently need blocks out of this).
  Eigen::MatrixXd L_;
  // Cache a few more in case we want to have a free end-constraint.
  Eigen::MatrixXd M_pinv_;
  Eigen::MatrixXd A_;
};

}  //  namespace loco_planner

#include "loco_planner/impl/loco_impl.h"

#endif  // LOCO_PLANNER_LOCO_H_
