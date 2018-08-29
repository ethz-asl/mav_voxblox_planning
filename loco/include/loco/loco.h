#ifndef LOCO_LOCO_H_
#define LOCO_LOCO_H_

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <Eigen/Core>

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
    // TODO(helenol): remove?
    double min_collision_sampling_dt = 0.1;
    double map_resolution = 0.1;  // Size of voxels in the map.
    bool verbose = false;
  }

  typedef std::function<double(const Eigen::VectorXd& position)>
      DistanceFunctionType;
  typedef std::function<double(const Eigen::VectorXd& position,
                               Eigen::VectorXd* gradient)>
      DistanceAndGradientFunctionType;

  Loco(size_t dimension);
  Loco(size_t dimension, const Config& config);

  // Set parameters. Defaults are okay, but these should be modifiable.
  double getEpsilon() const { return epsilon_; }
  void setEpsilon(double epsilon) { epsilon_ = epsilon; }
  double getBoundingSphereRadius() const { return bounding_sphere_radius_; }
  void setBoundingSphereRadius(double bounding_sphere_radius) {
    bounding_sphere_radius_ = bounding_sphere_radius;
  }

  double getWd() const { return w_d_; }
  void setWd(double w_d) { w_d_ = w_d; }
  double getWc() const { return w_c_; }
  void setWc(double w_c) { w_c_ = w_c; }
  double getCollisionSamplingDt() const { return collision_sampling_dt_; }
  void setCollisionSamplingDt(double dt) { collision_sampling_dt_ = dt; }
  double getMapResolution() const { return map_resolution_; }
  void setMapResolution(double map_resolution) {
    map_resolution_ = map_resolution;
  }
  bool getSoftGoalConstraint() const { return soft_goal_constraint_; }
  void setSoftGoalConstraint(bool soft_goal) {
    soft_goal_constraint_ = soft_goal;
  }

  // Should this have a copy of the world, or just ties to functions?
  // Let's start with ties to the functions.
  // Default param for time for testing.
  void setupFromPositions(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal, size_t num_segments,
                          double time = 10);

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
  // Only defined for 3D case.
  void getSampledSolution(double dt, std::vector<Eigen::Vector3d>* samples);
  double getCost();

  // Updates the free parameters of the underlying optimization.
  void setFreeDerivatives(const std::vector<Eigen::VectorXd>& d_p);
  void getFreeDerivatives(std::vector<Eigen::VectorXd>* d_p) const;
  void setParameterVector(const Eigen::VectorXd& parameters);
  void getParameterVector(Eigen::VectorXd* parameters) const;
  int getK() const { return K_; }
  int getNumParams() const { return num_free_; }

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

  // Output [t, p_x, p_y, ...] for MATLAB plotting.
  void printMatlabSampledTrajectory() const;

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
    // Should this be const? I DUNNO.
    Loco* parent_;
  };

  // This is where you store the matrices.
  mav_trajectory_generation::PolynomialOptimization<N> poly_opt_;

  // Collision cost functions.
  DistanceFunctionType distance_function_;
  // TODO.

  // Most of the configuration settings for the optimization.
  Config config_;

  // Params for the collision function.
  double epsilon_;
  double bounding_sphere_radius_;
  // Weights for deriv and collision costs.
  double w_d_;
  double w_c_;
  double w_g_;
  double collision_sampling_dt_;
  double map_resolution_;
  bool verbose_;
  bool soft_goal_constraint_;  // Whether to leave the end-position free.

  // If soft goal constraint is set, then save the goal location.
  Eigen::VectorXd goal_pos_;

  // Cache these params.
  int K_;  // Dimension of the problem.
  int num_free_;
  int num_fixed_;

  // Cache of all the relevant matrices.
  // These only depend on the segment time and number of free params.
  // Are these actually different per axis??? Nope they're all the same.
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

template <int N>
void Loco<N>::setupFromPositions(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal,
                                 size_t num_segments, double total_time) {
  mav_msgs::EigenTrajectoryPoint start_point;
  mav_msgs::EigenTrajectoryPoint goal_point;
  start_point.position_W = start;
  goal_point.position_W = goal;

  setupFromTrajectoryPoints(start_point, goal_point, num_segments, total_time);
}

template <int N>
void Loco<N>::setupFromTrajectoryPoints(
    const mav_msgs::EigenTrajectoryPoint& start_point,
    const mav_msgs::EigenTrajectoryPoint& goal_point, size_t num_segments,
    double total_time) {
  mav_trajectory_generation::timing::Timer timer_setup("loco/setup");

  std::vector<double> times(num_segments, total_time / num_segments);

  int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::JERK;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_segments + 1, mav_trajectory_generation::Vertex(K_));

  vertices.front().makeStartOrEnd(0, derivative_to_optimize);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      start_point.position_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      start_point.velocity_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      start_point.acceleration_W);
  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      goal_point.position_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      goal_point.velocity_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      goal_point.acceleration_W);
  poly_opt_.setupFromVertices(vertices, times, derivative_to_optimize);

  // Get the initial solution.
  poly_opt_.solveLinear();

  // If we're doing soft constraints, then get the current solution, remove the
  // constraint, and then feed it back as the initial condition.
  if (soft_goal_constraint_) {
    goal_pos_ = goal_point.position_W;
    // Just get the p from the segments, no need to redo this work.
    mav_trajectory_generation::Segment::Vector segments;
    poly_opt_.getSegments(&segments);
    std::vector<Eigen::VectorXd> p(K_, Eigen::VectorXd(N * segments.size()));

    for (int i = 0; i < K_; ++i) {
      for (size_t j = 0; j < segments.size(); ++j) {
        p[i].segment<N>(j * N) = segments[j][i].getCoefficients(0);
      }
    }

    // Now remove the goal position constraint.
    vertices.back().removeConstraint(
        mav_trajectory_generation::derivative_order::POSITION);
    poly_opt_.setupFromVertices(vertices, times, derivative_to_optimize);

    // Ok so fixed constraints are the same except the one we removed, just
    // have to properly pack the free constraints.
    poly_opt_.getA(&A_);
    poly_opt_.getMpinv(&M_pinv_);

    Eigen::VectorXd d_all(poly_opt_.getNumberFixedConstraints() +
                          poly_opt_.getNumberFreeConstraints());
    std::vector<Eigen::VectorXd> d_p(
        K_, Eigen::VectorXd(poly_opt_.getNumberFreeConstraints()));
    for (int i = 0; i < K_; ++i) {
      d_all = M_pinv_ * A_ * p[i];
      d_p[i] = d_all.tail(poly_opt_.getNumberFreeConstraints());
    }
    poly_opt_.setFreeConstraints(d_p);
  }

  // Allocate all the matrices and stuff.
  setupProblem();

  timer_setup.Stop();
}

template <int N>
void Loco<N>::setupProblem() {
  // Assume everything is computed by the underlying polyopt already.
  // We just need to copy out a copy for our uses...
  num_free_ = poly_opt_.getNumberFreeConstraints();
  num_fixed_ = poly_opt_.getNumberFixedConstraints();

  poly_opt_.getR(&R_);
  poly_opt_.getM(&M_);
  poly_opt_.getAInverse(&A_inv_);

  // Set up V as well... Eh later.
  size_t num_segments = poly_opt_.getNumberSegments();

  // V is an upper diagonal consisting of repeated 1:N-1s. It's square.
  V_.resize(num_segments * N, num_segments * N);
  V_.setZero();
  for (int i = 0; i < V_.diagonal(1).size(); ++i) {
    V_.diagonal(1)(i) = (i + 1) % N;
  }

  // Cache L while we're at it.
  L_ = A_inv_ * M_;
}

template <int N>
void Loco<N>::solveProblem() {
  // Find the current params.
  mav_trajectory_generation::timing::Timer timer_solve("loco/solve");

  const bool use_ceres = true;
  if (use_ceres) {
    solveProblemCeres();
  } else {
    solveProblemGradientDescent();
  }

  timer_solve.Stop();
}

template <int N>
void Loco<N>::solveProblemGradientDescent() {
  // Get initial parameters.
  std::vector<Eigen::VectorXd> grad_vec;

  Eigen::VectorXd x, grad, increment;
  getParameterVector(&x);
  grad.resize(x.size());
  grad.setZero();
  increment = grad;

  int max_iter = 50;
  double lambda = 10 * (w_c_ + w_d_);

  double cost = 0;
  for (int i = 0; i < max_iter; ++i) {
    // Evaluate cost.
    cost = computeTotalCostAndGradients(&grad_vec);

    // Unpack gradients.
    for (int k = 0; k < K_; ++k) {
      grad.segment(k * num_free_, num_free_) = grad_vec[k];
    }
    double step_size = 1.0 / (lambda + i);
    increment = -step_size * grad;
    std::cout << "[GD] i: " << i << " step size: " << step_size
              << " cost: " << cost << " gradient norm: " << grad.norm()
              << std::endl;

    // Update the parameters.
    x += increment;
    setParameterVector(x);
  }

  std::cout << "[Solution]: " << x.transpose() << std::endl;
}

template <int N>
void Loco<N>::solveProblemCeres() {
  std::vector<Eigen::VectorXd> d_p_vec;
  poly_opt_.getFreeConstraints(&d_p_vec);

  double* x0 = new double[num_free_ * K_];

  int i = 0;
  for (int k = 0; k < K_; ++k) {
    for (int j = 0; j < num_free_; ++j) {
      x0[i] = d_p_vec[k](j);
      ++i;
    }
  }

  // Create an object.
  ceres::GradientProblem problem(new NestedCeresFunction(K_, num_free_, this));

  ceres::GradientProblemSolver::Options options;
  options.line_search_direction_type = ceres::BFGS;
  // options.max_lbfgs_rank = 5;
  if (verbose_) {
    options.minimizer_progress_to_stdout = true;
  }
  // options.max_num_line_search_step_size_iterations = 4;
  options.line_search_interpolation_type = ceres::BISECTION;
  ceres::GradientProblemSolver::Summary summary;

  // Fire up CERES!
  ceres::Solve(options, problem, x0, &summary);

  if (verbose_) {
    std::cout << summary.FullReport() << "\n";
  }

  // Get the solution.
  i = 0;
  for (int k = 0; k < K_; ++k) {
    for (int j = 0; j < num_free_; ++j) {
      d_p_vec[k](j) = x0[i];
      ++i;
    }
  }
  // Put the solution BACK INTO THE SOLVER this is IMPORTANT!
  poly_opt_.setFreeConstraints(d_p_vec);

  if (verbose_) {
    std::cout << "[Solution]: " << d_p_vec[0].transpose() << " "
              << d_p_vec[1].transpose() << std::endl;
  }
}

template <int N>
void Loco<N>::setFreeDerivatives(const std::vector<Eigen::VectorXd>& d_p) {
  poly_opt_.setFreeConstraints(d_p);
}

template <int N>
void Loco<N>::getFreeDerivatives(std::vector<Eigen::VectorXd>* d_p) const {
  poly_opt_.getFreeConstraints(*d_p);
}

template <int N>
void Loco<N>::setParameterVector(const Eigen::VectorXd& parameters) {
  // Re-pack.
  std::vector<Eigen::VectorXd> d_p(K_, Eigen::VectorXd(num_free_));
  for (int k = 0; k < K_; ++k) {
    d_p[k] = parameters.segment(num_free_ * k, num_free_);
  }

  poly_opt_.setFreeConstraints(d_p);
}

template <int N>
void Loco<N>::getParameterVector(Eigen::VectorXd* parameters) const {
  // Re-pack.
  std::vector<Eigen::VectorXd> d_p(K_, Eigen::VectorXd(num_free_));
  poly_opt_.getFreeConstraints(&d_p);

  parameters->resize(K_ * num_free_);
  for (int k = 0; k < K_; ++k) {
    parameters->segment(k * num_free_, num_free_) = d_p[k];
  }
}

template <int N>
void Loco<N>::getSolution(mav_trajectory_generation::Trajectory* trajectory) {
  poly_opt_.getTrajectory(trajectory);
}

template <int N>
double Loco<N>::computeTotalCostAndGradients(
    std::vector<Eigen::VectorXd>* gradients) {
  std::vector<Eigen::VectorXd> grad_d;
  std::vector<Eigen::VectorXd> grad_c;
  std::vector<Eigen::VectorXd> grad_g;

  double J_d = 0.0, J_c = 0.0, J_g = 0.0;

  mav_trajectory_generation::timing::Timer timer_cost_grad_d(
      "loco/cost_grad_d");
  if (gradients != NULL) {
    J_d = computeDerivativeCostAndGradient(&grad_d);
  } else {
    J_d = computeDerivativeCostAndGradient(NULL);
  }
  timer_cost_grad_d.Stop();
  mav_trajectory_generation::timing::Timer timer_cost_grad_c(
      "loco/cost_grad_c");
  if (gradients != NULL) {
    J_c = computeCollisionCostAndGradient(&grad_c);
  } else {
    J_c = computeCollisionCostAndGradient(NULL);
  }
  timer_cost_grad_c.Stop();

  if (soft_goal_constraint_) {
    mav_trajectory_generation::timing::Timer timer_cost_grad_g(
        "loco/cost_grad_g");
    if (gradients != NULL) {
      J_g = computeGoalCostAndGradient(&grad_g);
    } else {
      J_g = computeGoalCostAndGradient(NULL);
    }
    timer_cost_grad_g.Stop();
  }

  double cost = w_d_ * J_d + w_c_ * J_c + w_g_ * J_g;

  // Add the gradients too...
  if (gradients != NULL) {
    gradients->clear();
    gradients->resize(K_, Eigen::VectorXd::Zero(num_free_));
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k] = w_d_ * grad_d[k] + w_c_ * grad_c[k];
      if (soft_goal_constraint_ && !grad_g.empty()) {
        (*gradients)[k] += w_g_ * grad_g[k];
      }
    }
  }
  return cost;
}

template <int N>
double Loco<N>::computeDerivativeCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) {
  // Compare the two approaches:
  // getCost() and the full matrix.
  double J_d = 0;
  std::vector<Eigen::VectorXd> grad_d(K_, Eigen::VectorXd::Zero(num_free_));

  // Set up mappings to R_FF R_FP R_PP etc. R_FP' = R_PF if that saves
  // time eventually.
  // All of these are the same per axis.
  // Not sure if there's a boost from the sparse solver? Our problems are tiny
  // so I guess not.

  // R_ff * d_f is actually constant so can cache this term.
  Eigen::Block<Eigen::MatrixXd> R_ff = R_.block(0, 0, num_fixed_, num_fixed_);

  Eigen::Block<Eigen::MatrixXd> R_pf =
      R_.block(num_fixed_, 0, num_free_, num_fixed_);

  Eigen::Block<Eigen::MatrixXd> R_pp =
      R_.block(num_fixed_, num_fixed_, num_free_, num_free_);

  // Get d_p and d_f vector for all axes.
  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;

  // TODO(helenol): figure out if we should have polyopt keep track of d_ps
  // or us keep track of d_ps over iterations.
  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  Eigen::MatrixXd J_d_temp;
  // Compute costs over all axes.
  for (int k = 0; k < K_; ++k) {
    // Get a copy of d_p and d_f for this axis.
    const Eigen::VectorXd& d_p = d_p_vec[k];
    const Eigen::VectorXd& d_f = d_f_vec[k];

    // std::cout << "d_p:\n" << d_p << "\n\nd_f:\n" << d_f << std::endl;

    // Now do the other thing.
    J_d_temp = d_f.transpose() * R_ff * d_f +
               d_f.transpose() * R_pf.transpose() * d_p +
               d_p.transpose() * R_pf * d_f + d_p.transpose() * R_pp * d_p;
    J_d += J_d_temp(0, 0);

    // And get the gradient.
    // Should really separate these out by k.
    grad_d[k] =
        2 * d_f.transpose() * R_pf.transpose() + 2 * d_p.transpose() * R_pp;
  }

  double cost = J_d;
  if (gradients != NULL) {
    gradients->clear();
    gradients->resize(K_);
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k] = grad_d[k];
    }
  }
  return cost;
}

template <int N>
double Loco<N>::computeCollisionCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) {
  // Unpack into d_f, d_ps.
  // TODO: still figure out who should know about d_ps.
  // Get d_p and d_f vector for all axes.
  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;

  // TODO(helenol): figure out if we should have polyopt keep track of d_ps
  // or us keep track of d_ps over iterations.
  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  // Get a vector of ps.
  size_t num_segments = poly_opt_.getNumberSegments();

  std::vector<Eigen::VectorXd> p_vec(K_, Eigen::VectorXd(N * num_segments));
  for (int k = 0; k < K_; ++k) {
    Eigen::VectorXd d_all(num_fixed_ + num_free_);
    d_all.head(num_fixed_) = d_f_vec[k];
    d_all.tail(num_free_) = d_p_vec[k];

    // Get the coefficients out.
    // L is shorthand for A_inv * M.
    p_vec[k] = L_ * d_all;
  }

  // Sample with some dt.
  // Actually how do we store the segment times????
  // Get them out of the polynomial optimization.
  // Probably don't need to copy them out each time.
  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);

  // Get the correct L block to calculate derivatives.
  Eigen::Block<Eigen::MatrixXd> L_pp =
      L_.block(0, num_fixed_, L_.rows(), num_free_);

  // Yeah really gotta fill these in from real parameters.
  double dt = collision_sampling_dt_;
  double distance_int_limit = map_resolution_;

  // Could probably do something more intelligent here as well.
  // But general idea: evaluate at a time, see what the distance is, if it's
  // far enough from the last point, just evalute the gradient.
  double J_c = 0;
  std::vector<Eigen::VectorXd> grad_c(K_, Eigen::VectorXd::Zero(num_free_));

  Eigen::VectorXd last_position(K_);
  last_position.setZero();
  // int is "integral" in this case, not "integer."
  double time_int = -1;
  double distance_int = 0;
  double t = 0.0;
  for (int i = 0; i < num_segments; ++i) {
    // Select a time.
    // std::cout << "Starting segment " << i << std::endl;
    for (t = 0.0; t < segment_times[i]; t += dt) {
      // T is the vector for just THIS SEGMENT.
      Eigen::VectorXd T_seg(N);
      getTVector(t, &T_seg);

      // Now fill this in for ALL segments.
      Eigen::VectorXd T(num_segments * N);
      T.setZero();
      T.segment(i * N, N) = T_seg;

      // std::cout << "T: " << T.transpose() << std::endl;

      // Calculate the position per axis. Also calculate velocity so we don't
      // have to get p_k_i out again.
      Eigen::VectorXd position(K_);
      Eigen::VectorXd velocity(K_);
      position.setZero();
      for (int k = 0; k < K_; ++k) {
        // Get the coefficients just for this segment.
        // TODO(helenol): OPTIMIZE!
        Eigen::Block<Eigen::VectorXd> p_k_i = p_vec[k].block(i * N, 0, N, 1);
        position(k) = (T_seg.transpose() * p_k_i)(0);
        velocity(k) = (T_seg.transpose() * V_.block(0, 0, N, N) * p_k_i)(0);
      }

      // Now calculate the distance integral.
      if (time_int < 0) {
        // Skip this entry if it's the first one.
        time_int = 0.0;
        last_position = position;
        continue;
      }
      time_int += dt;
      distance_int += (position - last_position).norm();
      last_position = position;
      // Don't need to evaluate anything at this position.
      if (distance_int < distance_int_limit) {
        continue;
      }

      // Okay figure out the cost and gradient of the potential map at this
      // point.
      Eigen::VectorXd d_c_d_f(K_);

      mav_trajectory_generation::timing::Timer timer_map_lookup(
          "loco/map_lookup");
      double c = 0;
      if (gradients != NULL) {
        c = computePotentialCostAndGradient(position, &d_c_d_f);
      } else {
        c = computePotentialCostAndGradient(position, NULL);
      }
      timer_map_lookup.Stop();

      double cost = c * velocity.norm() * time_int;

      J_c += cost;

      if (gradients != NULL) {
        // Gotta make sure the norm is non-zero, since we divide by it later.
        if (velocity.norm() > 1e-6) {
          // Now calculate the gradient per axis.
          for (int k = 0; k < K_; ++k) {
            Eigen::VectorXd grad_c_k =
                (velocity.norm() * time_int * d_c_d_f(k) * T.transpose() *
                     L_pp +
                 time_int * c * velocity(k) / velocity.norm() * T.transpose() *
                     V_ * L_pp)
                    .transpose();

            grad_c[k] += grad_c_k;
          }
        }
      }

      // Clear the numeric integrals.
      distance_int = 0.0;
      time_int = 0.0;
      last_position = position;
    }
    // Make sure the dt is correct for the next step:
    time_int += -dt + (segment_times[i] - t);
  }

  if (gradients != NULL) {
    gradients->clear();
    gradients->resize(K_, Eigen::VectorXd(num_free_));
    *gradients = grad_c;
  }
  return J_c;
}

template <int N>
double Loco<N>::computeGoalCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) {
  // Ok we just care about the T of the last point in the last segment.
  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);
  size_t num_segments = segment_times.size();
  Eigen::VectorXd T_last_seg;
  getTVector(segment_times.back(), &T_last_seg);

  // I guess the best is to just pad out T_g with 0s up to last seg.
  Eigen::VectorXd T(num_segments * N);
  T.setZero();
  T.segment<N>((num_segments - 1) * N) = T_last_seg;

  // Get d_p and d_f vector for all axes.
  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;

  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  // Unpack the ps.
  std::vector<Eigen::VectorXd> p_vec(K_, Eigen::VectorXd(N * num_segments));
  double J_g = 0.0;

  // Get the correct L block to calculate derivatives.
  Eigen::Block<Eigen::MatrixXd> L_pp =
      L_.block(0, num_fixed_, L_.rows(), num_free_);

  Eigen::VectorXd actual_goal_pos = Eigen::VectorXd::Zero(K_);
  for (int k = 0; k < K_; ++k) {
    Eigen::VectorXd d_all(num_fixed_ + num_free_);
    d_all.head(num_fixed_) = d_f_vec[k];
    d_all.tail(num_free_) = d_p_vec[k];

    // Get the coefficients out.
    // L is shorthand for A_inv * M.
    p_vec[k] = L_ * d_all;

    actual_goal_pos(k) = T.transpose() * p_vec[k];
  }

  J_g = (actual_goal_pos - goal_pos_).norm();

  // Fill in gradients too.
  if (gradients != NULL) {
    gradients->resize(K_);
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k].resize(num_free_);
      (*gradients)[k].setZero();
      Eigen::MatrixXd df_dpk(K_, num_segments * N);
      df_dpk.setZero();
      df_dpk.row(k) = T;
      (*gradients)[k] =
          ((actual_goal_pos - goal_pos_) / J_g).transpose() * df_dpk * L_pp;
    }
  }

  return J_g;
}

template <int N>
void Loco<N>::getTVector(double t, Eigen::VectorXd* T) const {
  T->resize(N);
  for (int i = 0; i < N; ++i) {
    (*T)(i) = pow(t, i);
  }
}

template <int N>
void Loco<N>::printMatlabSampledTrajectory() const {
  // Allocate some size of p vector.
  double dt = 0.1;

  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);

  int num_segments = poly_opt_.getNumberSegments();
  int total_samples = 0;
  for (int i = 0; i < num_segments; ++i) {
    total_samples += static_cast<int>(std::ceil(segment_times[i] / dt)) + 1;
  }

  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;
  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  std::vector<Eigen::VectorXd> p_vec(K_,
                                     Eigen::VectorXd::Zero(N * num_segments));
  for (int k = 0; k < K_; ++k) {
    Eigen::VectorXd d_all(num_fixed_ + num_free_);
    d_all.head(num_fixed_) = d_f_vec[k];
    d_all.tail(num_free_) = d_p_vec[k];

    // Get the coefficients out.
    // L is shorthand for A_inv * M.
    p_vec[k] = L_ * d_all;
  }

  // Layout: [t, x, y, z...]
  Eigen::MatrixXd output(total_samples, K_ + 1);
  output.setZero();
  int j = 0;

  double t = 0.0;
  double current_segment_time = 0.0;
  for (int i = 0; i < num_segments; ++i) {
    // Select a time.
    for (t = 0.0; t < segment_times[i]; t += dt) {
      // T is the vector for just THIS SEGMENT.
      Eigen::VectorXd T_seg(N);
      // FILL THIS IN.
      getTVector(t, &T_seg);

      // Calculate the position per axis. Also calculate velocity so we don't
      // have to get p_k_i out again.
      Eigen::VectorXd position(K_);
      position.setZero();
      for (int k = 0; k < K_; ++k) {
        // Get the coefficients just for this segment.
        // TODO(helenol): OPTIMIZE!
        Eigen::Block<Eigen::VectorXd> p_k_i = p_vec[k].block(i * N, 0, N, 1);
        position(k) = (T_seg.transpose() * p_k_i)(0);
      }
      if (j < output.rows()) {
        output(j, 0) = t + current_segment_time;
        output.row(j).segment(1, K_) = position.transpose();
        j++;
      }
    }
    current_segment_time += segment_times[i];
  }

  Eigen::IOFormat matlab_format(Eigen::StreamPrecision, 0, ", ", ";\n", "", "",
                                "[", "]");
  std::cout << output.row(0).segment(1, K_) << std::endl;
  std::cout << "\n----------------------------------------\n";
  std::cout << output.format(matlab_format);
  std::cout << "\n\n";
}

template <int N>
double Loco<N>::potentialFunction(double distance) const {
  double result = 0.0;
  distance -= bounding_sphere_radius_;
  if (distance < 0) {
    result = -distance + 0.5 * epsilon_;
  } else if (distance <= epsilon_) {
    double epsilon_distance = distance - epsilon_;
    result = 0.5 * 1.0 / epsilon_ * epsilon_distance * epsilon_distance;
  } else {
    result = 0.0;
  }
  return result;
}

template <int N>
double Loco<N>::computePotentialCostAndGradient(const Eigen::VectorXd& position,
                                                Eigen::VectorXd* gradient) {
  Eigen::VectorXd distance_gradient(K_);
  distance_gradient.setZero();
  Eigen::VectorXd increment(K_);

  double d = distance_function_(position);
  double c = potentialFunction(d);
  // Get numeric derivative for each dim.
  if (gradient != NULL) {
    for (int k = 0; k < K_; ++k) {
      // Gradient computations for a single dimension.
      increment.setZero();
      increment(k) = map_resolution_;
      double left_distance =
          potentialFunction(distance_function_(position - increment));
      double right_distance =
          potentialFunction(distance_function_(position + increment));
      double dim_gradient =
          (right_distance - left_distance) / (2.0 * map_resolution_);
      distance_gradient(k) = dim_gradient;
    }

    *gradient = distance_gradient;
  }
  return c;
}

template <int N>
bool Loco<N>::NestedCeresFunction::Evaluate(const double* parameters,
                                            double* cost,
                                            double* gradient) const {
  // Step 1: allocate all the necessary Eigen vectors.
  std::vector<Eigen::VectorXd> d_p(K_, Eigen::VectorXd::Zero(num_free_));
  std::vector<Eigen::VectorXd> grad_vec(K_, Eigen::VectorXd::Zero(num_free_));

  // Step 2: unpack the parameters into d_p.
  int i = 0;
  for (int k = 0; k < K_; ++k) {
    for (int j = 0; j < num_free_; ++j) {
      d_p[k](j) = parameters[i];
      ++i;
    }
  }

  // Step 3: set the d_p of the underlying problem.
  parent_->setFreeDerivatives(d_p);

  // Step 4: compute costs and gradients.
  *cost = parent_->computeTotalCostAndGradients(&grad_vec);

  // Step 5: re-pack gradients back into flat format.
  if (gradient != NULL) {
    i = 0;
    for (int k = 0; k < K_; ++k) {
      for (int j = 0; j < num_free_; ++j) {
        gradient[i] = grad_vec[k](j);
        ++i;
      }
    }
  }

  return true;
}
template <int N>
int Loco<N>::NestedCeresFunction::NumParameters() const {
  return num_free_ * K_;
}

template <int N>
void Loco<N>::getSampledSolution(double dt,
                                 std::vector<Eigen::Vector3d>* samples) {
  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);

  int num_segments = poly_opt_.getNumberSegments();
  int total_samples = 0;
  for (int i = 0; i < num_segments; ++i) {
    total_samples += static_cast<int>(std::ceil(segment_times[i] / dt)) + 1;
  }

  samples->clear();
  samples->reserve(total_samples);

  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;
  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  std::vector<Eigen::VectorXd> p_vec(K_,
                                     Eigen::VectorXd::Zero(N * num_segments));
  for (int k = 0; k < K_; ++k) {
    Eigen::VectorXd d_all(num_fixed_ + num_free_);
    d_all.head(num_fixed_) = d_f_vec[k];
    d_all.tail(num_free_) = d_p_vec[k];

    // Get the coefficients out.
    // L is shorthand for A_inv * M.
    p_vec[k] = L_ * d_all;
  }

  // Layout: [t, x, y, z...]
  Eigen::MatrixXd output(total_samples, K_ + 1);
  output.setZero();

  double t = 0.0;
  double current_segment_time = 0.0;
  for (int i = 0; i < num_segments; ++i) {
    // Select a time.
    for (t = 0.0; t < segment_times[i]; t += dt) {
      // T is the vector for just THIS SEGMENT.
      Eigen::VectorXd T_seg(N);
      // FILL THIS IN.
      getTVector(t, &T_seg);

      // Calculate the position per axis. Also calculate velocity so we don't
      // have to get p_k_i out again.
      Eigen::VectorXd position(K_);
      position.setZero();
      for (int k = 0; k < K_; ++k) {
        // Get the coefficients just for this segment.
        // TODO(helenol): OPTIMIZE!
        Eigen::Block<Eigen::VectorXd> p_k_i = p_vec[k].block(i * N, 0, N, 1);
        position(k) = (T_seg.transpose() * p_k_i)(0);
      }
      samples->push_back(position);
    }
    current_segment_time += segment_times[i];
  }
}

template <int N>
double Loco<N>::getCost() {
  return computeTotalCostAndGradients(NULL);
}

}  //  namespace loco

#include "loco/impl/loco_impl.h"

#endif  // LOCO_LOCO_H_
