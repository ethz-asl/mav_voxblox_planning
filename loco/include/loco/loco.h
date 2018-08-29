#ifndef LOCO_LOCO_H_
#define LOCO_LOCO_H_

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <functional>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/timing.h>

namespace loco {

// Templated on the same as the polynomial optimization.
// N coefficients, describing a polynomial of order N-1.
template <int N = 10>
class Loco {
  static_assert(N % 2 == 0, "The number of coefficients has to be even.");

 public:
  struct Config {
    double epsilon = 0.5;
    double robot_radius = 0.5;
    bool soft_goal_constraint = true;
    double w_d = 0.1;   // Smoothness cost weight.
    double w_c = 10.0;  // Collision cost weight.
    double w_g = 2.5;   // Soft goal cost weight (if using soft goals).
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
  void setupProblem();

  // This should probably return something...
  void solveProblem();

  // Different solver methods.
  void solveProblemCeres();
  void solveProblemGradientDescent();

  // Accessors for getting the data back out...
  void getSolution(mav_trajectory_generation::Trajectory* trajectory);

  // The final cost in the optimization.
  double getCost();

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
  double getWd() const { return config_.w_d_; }
  void setWd(double w_d) { config_.w_d_ = w_d; }
  double getWc() const { return config_.w_c_; }
  void setWc(double w_c) { config_.w_c_ = w_c; }
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

  // Internal functions, for testing, debugging, or advanced use.

  // Returns cost and gradient vector, with respect to k (separate for each
  // dimension).
  double computeTotalCostAndGradients(std::vector<Eigen::VectorXd>* gradients);
  // Just the J_d part.
  double computeDerivativeCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients);
  // Just the J_c part.
  double computeCollisionCostAndGradient(
      std::vector<Eigen::VectorXd>* gradients);
  // The J_g part if using soft goals.
  double computeGoalCostAndGradient(std::vector<Eigen::VectorXd>* gradients);

  double computePotentialCostAndGradient(const Eigen::VectorXd& position,
                                         Eigen::VectorXd* gradient);
  double potentialFunction(double distance) const;

  // Set how to get the distance of a point.
  void setDistanceFunction(const DistanceFunctionType& function) {
    distance_function_ = function;
  }

  // Convenience.
  void getTVector(double t, Eigen::VectorXd* T) const;

 private:
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

}  //  namespace loco

#include "loco/impl/loco_impl.h"

#endif  // LOCO_LOCO_H_
