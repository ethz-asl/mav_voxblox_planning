#ifndef LOCO_PLANNER_IMPL_LOCO_IMPL_H_
#define LOCO_PLANNER_IMPL_LOCO_IMPL_H_

namespace loco_planner {

template <int N>
Loco<N>::Loco(size_t dimension) : Loco(dimension, Config()) {}

template <int N>
Loco<N>::Loco(size_t dimension, const Config& config)
    : poly_opt_(dimension), K_(dimension), num_free_(0) {}

template <int N>
void Loco<N>::setupFromPositions(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal,
                                 size_t num_segments, double total_time) {
  mav_trajectory_generation::Vertex::Vector vertices(
      num_segments + 1, mav_trajectory_generation::Vertex(K_));

  vertices.front().makeStartOrEnd(
      0, mav_trajectory_generation::getHighestDerivativeFromN(N));
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION, start);
  vertices.back().makeStartOrEnd(
      0, mav_trajectory_generation::getHighestDerivativeFromN(N));
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION, goal);

  setupFromVertices(total_time, &vertices);
}

template <int N>
void Loco<N>::setupFromTrajectoryPoints(
    const mav_msgs::EigenTrajectoryPoint& start_point,
    const mav_msgs::EigenTrajectoryPoint& goal_point, size_t num_segments,
    double total_time) {
  mav_trajectory_generation::Vertex::Vector vertices(
      num_segments + 1, mav_trajectory_generation::Vertex(K_));

  vertices.front().makeStartOrEnd(
      0, mav_trajectory_generation::getHighestDerivativeFromN(N));
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      start_point.position_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      start_point.velocity_W);
  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      start_point.acceleration_W);
  vertices.back().makeStartOrEnd(
      0, mav_trajectory_generation::getHighestDerivativeFromN(N));
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      goal_point.position_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      goal_point.velocity_W);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      goal_point.acceleration_W);

  setupFromVertices(total_time, &vertices);
}

// Vertices can be modified by this function.
template <int N>
void Loco<N>::setupFromVertices(
    double total_time, mav_trajectory_generation::Vertex::Vector* vertices) {
  mav_trajectory_generation::timing::Timer timer_setup("loco/setup");

  std::vector<double> times((vertices->size() - 1),
                            total_time / (vertices->size() - 1));

  poly_opt_.setupFromVertices(*vertices, times, config_.derivative_to_optimize);

  // Get the initial solution.
  poly_opt_.solveLinear();

  // If we're doing soft constraints, then get the current solution, remove the
  // constraint, and then feed it back as the initial condition.
  if (config_.soft_goal_constraint) {
    vertices->back().getConstraint(
        mav_trajectory_generation::derivative_order::POSITION, &goal_pos_);
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
    vertices->back().removeConstraint(
        mav_trajectory_generation::derivative_order::POSITION);
    poly_opt_.setupFromVertices(*vertices, times,
                                config_.derivative_to_optimize);

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

  setupProblem();

  timer_setup.Stop();
}

template <int N>
void Loco<N>::setupFromTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory) {
  mav_trajectory_generation::timing::Timer timer_setup("loco/setup");
  // Luckily the trajectory contains most of what we need.
  std::vector<double> times = trajectory.getSegmentTimes();
  mav_trajectory_generation::Vertex::Vector vertices;
  vertices.reserve(times.size() + 1);

  vertices.emplace_back(trajectory.getStartVertex(
      mav_trajectory_generation::getHighestDerivativeFromN(N)));
  for (size_t i = 0; i < times.size() - 1; ++i) {
    vertices.emplace_back(mav_trajectory_generation::Vertex(K_));
  }
  vertices.emplace_back(trajectory.getGoalVertex(
      mav_trajectory_generation::getHighestDerivativeFromN(N)));

  poly_opt_.setupFromVertices(vertices, times, config_.derivative_to_optimize);

  // Now we remove the constraints, and re-set all the free constraints. Same as
  // for the soft goal case.
  mav_trajectory_generation::Segment::Vector segments;
  trajectory.getSegments(&segments);
  std::vector<Eigen::VectorXd> p(K_, Eigen::VectorXd(N * segments.size()));

  for (int i = 0; i < K_; ++i) {
    for (size_t j = 0; j < segments.size(); ++j) {
      p[i].segment<N>(j * N) = segments[j][i].getCoefficients(0);
    }
  }

  // Now optionally remove the goal position constraint.
  if (config_.soft_goal_constraint) {
    vertices.back().getConstraint(
        mav_trajectory_generation::derivative_order::POSITION, &goal_pos_);
    vertices.back().removeConstraint(
        mav_trajectory_generation::derivative_order::POSITION);
  }

  poly_opt_.setupFromVertices(vertices, times, config_.derivative_to_optimize);

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

  setupProblem();

  timer_setup.Stop();
}

template <int N>
void Loco<N>::setupFromTrajectoryAndResample(
    const mav_trajectory_generation::Trajectory& trajectory,
    size_t num_segments) {
  mav_trajectory_generation::timing::Timer timer_setup("loco/setup");
  const int highest_derivative =
      mav_trajectory_generation::getHighestDerivativeFromN(N);
  // Luckily the trajectory contains most of what we need.
  double total_time = trajectory.getMaxTime();
  std::vector<double> times(num_segments, total_time / num_segments);
  mav_trajectory_generation::Vertex::Vector vertices;
  vertices.reserve(times.size() + 1);

  double time_so_far = 0.0;
  vertices.emplace_back(trajectory.getStartVertex(highest_derivative));
  for (size_t i = 0; i < times.size() - 1; ++i) {
    time_so_far += times[i];
    vertices.emplace_back(
        trajectory.getVertexAtTime(time_so_far, highest_derivative));
  }
  vertices.emplace_back(trajectory.getGoalVertex(highest_derivative));

  poly_opt_.setupFromVertices(vertices, times, config_.derivative_to_optimize);
  poly_opt_.solveLinear();

  // Now we remove the constraints, and re-set all the free constraints. Same as
  // for the soft goal case.
  mav_trajectory_generation::Segment::Vector segments;
  poly_opt_.getSegments(&segments);
  std::vector<Eigen::VectorXd> p(K_, Eigen::VectorXd(N * segments.size()));
  for (int i = 0; i < K_; ++i) {
    for (size_t j = 0; j < segments.size(); ++j) {
      p[i].segment<N>(j * N) = segments[j][i].getCoefficients(0);
    }
  }

  // Remove all the intermediate constraints.
  for (size_t i = 1; i < vertices.size() - 1; ++i) {
    for (int j = 0; j <= highest_derivative; ++j) {
      vertices[i].removeConstraint(j);
    }
  }

  // Now optionally remove the goal position constraint.
  if (config_.soft_goal_constraint) {
    vertices.back().getConstraint(
        mav_trajectory_generation::derivative_order::POSITION, &goal_pos_);
    vertices.back().removeConstraint(
        mav_trajectory_generation::derivative_order::POSITION);
  }

  poly_opt_.setupFromVertices(vertices, times, config_.derivative_to_optimize);

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

  setupProblem();

  timer_setup.Stop();
}

template <int N>
void Loco<N>::setWaypoints(const std::map<double, Eigen::VectorXd>& waypoints) {
  waypoints_ = waypoints;
}

template <int N>
void Loco<N>::setWaypointsFromTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory) {
  waypoints_.clear();
  std::vector<double> times = trajectory.getSegmentTimes();
  // Skip the first one, obviously doesn't make any sense since start is fixed.
  // Same with the goal, soft goal cost is separate.
  double time_so_far = 0.0;
  for (size_t i = 0; i < times.size() - 1; ++i) {
    time_so_far += times[i];
    waypoints_[time_so_far] = trajectory.evaluate(time_so_far);
  }
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
  solveProblemCeres();

  timer_solve.Stop();
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

  if (config_.verbose) {
    options.minimizer_progress_to_stdout = true;
  }
  options.line_search_interpolation_type = ceres::BISECTION;
  ceres::GradientProblemSolver::Summary summary;

  // Fire up CERES!
  ceres::Solve(options, problem, x0, &summary);

  if (config_.verbose) {
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

  if (config_.verbose) {
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
void Loco<N>::getTrajectory(
    mav_trajectory_generation::Trajectory* trajectory) const {
  poly_opt_.getTrajectory(trajectory);
}

template <int N>
double Loco<N>::computeTotalCostAndGradients(
    std::vector<Eigen::VectorXd>* gradients) const {
  std::vector<Eigen::VectorXd> grad_d;
  std::vector<Eigen::VectorXd> grad_c;
  std::vector<Eigen::VectorXd> grad_g;
  std::vector<Eigen::VectorXd> grad_w;

  double J_d = 0.0, J_c = 0.0, J_g = 0.0, J_w = 0.0;

  mav_trajectory_generation::timing::Timer timer_cost_grad_d(
      "loco/cost_grad_d");
  if (gradients != nullptr) {
    J_d = computeDerivativeCostAndGradient(&grad_d);
  } else {
    J_d = computeDerivativeCostAndGradient(nullptr);
  }
  timer_cost_grad_d.Stop();
  mav_trajectory_generation::timing::Timer timer_cost_grad_c(
      "loco/cost_grad_c");
  if (gradients != nullptr) {
    J_c = computeCollisionCostAndGradient(&grad_c);
  } else {
    J_c = computeCollisionCostAndGradient(nullptr);
  }
  timer_cost_grad_c.Stop();

  if (config_.soft_goal_constraint) {
    mav_trajectory_generation::timing::Timer timer_cost_grad_g(
        "loco/cost_grad_g");
    if (gradients != nullptr) {
      J_g = computeGoalCostAndGradient(&grad_g);
    } else {
      J_g = computeGoalCostAndGradient(nullptr);
    }
    timer_cost_grad_g.Stop();
  }

  if (!waypoints_.empty()) {
    mav_trajectory_generation::timing::Timer timer_cost_grad_w(
        "loco/cost_grad_w");
    if (gradients != nullptr) {
      J_w = computeWaypointCostAndGradient(&grad_w);
    } else {
      J_w = computeWaypointCostAndGradient(nullptr);
    }
    timer_cost_grad_w.Stop();
  }

  double cost = config_.w_d * J_d + config_.w_c * J_c + config_.w_g * J_g +
                config_.w_w * J_w;

  // Add the gradients too...
  if (gradients != nullptr) {
    gradients->clear();
    gradients->resize(K_, Eigen::VectorXd::Zero(num_free_));
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k] = config_.w_d * grad_d[k] + config_.w_c * grad_c[k];
      if (config_.soft_goal_constraint && !grad_g.empty()) {
        (*gradients)[k] += config_.w_g * grad_g[k];
      }
      if (!grad_w.empty()) {
        (*gradients)[k] += config_.w_w * grad_w[k];
      }
    }
  }
  return cost;
}

template <int N>
double Loco<N>::computeDerivativeCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) const {
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
  const Eigen::Block<const Eigen::MatrixXd> R_ff =
      R_.block(0, 0, num_fixed_, num_fixed_);

  const Eigen::Block<const Eigen::MatrixXd> R_pf =
      R_.block(num_fixed_, 0, num_free_, num_fixed_);

  const Eigen::Block<const Eigen::MatrixXd> R_pp =
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
  if (gradients != nullptr) {
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
    std::vector<Eigen::VectorXd>* gradients) const {
  mav_trajectory_generation::timing::Timer coll_cost_prep_timer(
      "loco/coll_cost_prep");
  // Unpack into d_f, d_ps.
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
  Eigen::Block<const Eigen::MatrixXd> L_pp =
      L_.block(0, num_fixed_, L_.rows(), num_free_);

  double dt = config_.min_collision_sampling_dt;
  double distance_int_limit = config_.map_resolution;

  // Could probably do something more intelligent here as well.
  // But general idea: evaluate at a time, see what the distance is, if it's
  // far enough from the last point, just evalute the gradient.
  double J_c = 0;
  std::vector<Eigen::VectorXd> grad_c(K_, Eigen::VectorXd::Zero(num_free_));

  coll_cost_prep_timer.Stop();

  Eigen::VectorXd last_position(K_);
  last_position.setZero();
  // int is "integral" in this case, not "integer."
  double time_int = -1.0;
  double distance_int = 0;
  double t = 0.0;

  Eigen::VectorXd T(num_segments * N);
  Eigen::VectorXd T_seg(N);
  for (int i = 0; i < num_segments; ++i) {
    T.setZero();

    // Select a time.
    for (t = 0.0; t < segment_times[i]; t += dt) {
      mav_trajectory_generation::timing::Timer coll_cost_sample_timer(
          "loco/coll_cost_sample");

      // T is the vector for just THIS SEGMENT.
      getTVector(t, &T_seg);

      // Now fill this in for ALL segments.
      T.segment(i * N, N) = T_seg;

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
      if (time_int < 0.0) {
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

      coll_cost_sample_timer.Stop();

      // Okay figure out the cost and gradient of the potential map at this
      // point.
      Eigen::VectorXd d_c_d_f(K_);

      mav_trajectory_generation::timing::Timer timer_map_lookup(
          "loco/map_lookup");
      double c = 0.0;
      if (gradients != nullptr) {
        c = computePotentialCostAndGradient(position, &d_c_d_f);
      } else {
        c = computePotentialCostAndGradient(position, nullptr);
      }
      timer_map_lookup.Stop();

      double cost = c * velocity.norm() * time_int;

      J_c += cost;

      mav_trajectory_generation::timing::Timer coll_cost_grad_timer(
          "loco/coll_cost_grad");

      if (gradients != nullptr) {
        // Gotta make sure the norm is non-zero, since we divide by it later.
        if (velocity.norm() > 1e-6 && (cost > 0.0 || d_c_d_f.norm() > 0.0)) {
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

      coll_cost_grad_timer.Stop();

      // Clear the numeric integrals.
      distance_int = 0.0;
      time_int = 0.0;
      last_position = position;
    }
    // Make sure the dt is correct for the next step:
    time_int += -dt + (segment_times[i] - t);
  }

  if (gradients != nullptr) {
    gradients->clear();
    gradients->resize(K_, Eigen::VectorXd(num_free_));
    *gradients = grad_c;
  }
  return J_c;
}

template <int N>
double Loco<N>::computeGoalCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) const {
  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);
  double total_time =
      std::accumulate(segment_times.begin(), segment_times.end(), 0.0);

  return computePositionSoftCostAndGradient(total_time, goal_pos_, gradients);
}

template <int N>
double Loco<N>::computeWaypointCostAndGradient(
    std::vector<Eigen::VectorXd>* gradients) const {
  if (gradients != nullptr) {
    gradients->resize(K_);
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k].resize(num_free_);
      (*gradients)[k].setZero();
    }
  }

  if (waypoints_.empty()) {
    // TODO(helenol): Do I need to fill in gradients?
    return 0.0;
  }

  std::vector<Eigen::VectorXd> waypoint_gradient;
  double total_cost = 0.0;
  for (const std::pair<double, Eigen::VectorXd>& kv : waypoints_) {
    if (gradients == nullptr) {
      total_cost +=
          computePositionSoftCostAndGradient(kv.first, kv.second, nullptr);
    } else {
      total_cost += computePositionSoftCostAndGradient(kv.first, kv.second,
                                                       &waypoint_gradient);
      for (int k = 0; k < K_; ++k) {
        (*gradients)[k] += waypoint_gradient[k];
      }
    }
  }

  return total_cost;
}

template <int N>
double Loco<N>::computePositionSoftCostAndGradient(
    double t, const Eigen::VectorXd& position,
    std::vector<Eigen::VectorXd>* gradients) const {
  std::vector<double> segment_times;
  poly_opt_.getSegmentTimes(&segment_times);
  size_t num_segments = segment_times.size();

  // Find the correct segment for this to apply.
  double accumulated_time = 0.0;
  // Look for the correct segment.
  size_t i = 0;
  for (i = 0; i < segment_times.size(); ++i) {
    accumulated_time += segment_times[i];
    if (accumulated_time > t) {
      break;
    }
  }
  // Make sure we don't go off the end of the segments (can happen if t is
  // equal to trajectory max time).
  if (i >= segment_times.size()) {
    i = segment_times.size() - 1;
  }
  // Go back to the start of this segment.
  accumulated_time -= segment_times[i];
  int segment_index = i;
  double segment_time = accumulated_time;

  Eigen::VectorXd T_desired_seg;
  getTVector(segment_time, &T_desired_seg);

  // I guess the best is to just pad out T_g with 0s up to last seg.
  Eigen::VectorXd T(num_segments * N);
  T.setZero();
  T.segment<N>(segment_index * N) = T_desired_seg;

  // Get d_p and d_f vector for all axes.
  std::vector<Eigen::VectorXd> d_p_vec;
  std::vector<Eigen::VectorXd> d_f_vec;

  poly_opt_.getFreeConstraints(&d_p_vec);
  poly_opt_.getFixedConstraints(&d_f_vec);

  // Unpack the ps.
  std::vector<Eigen::VectorXd> p_vec(K_, Eigen::VectorXd(N * num_segments));
  double J_w = 0.0;

  // Get the correct L block to calculate derivatives.
  const Eigen::Block<const Eigen::MatrixXd> L_pp =
      L_.block(0, num_fixed_, L_.rows(), num_free_);

  Eigen::VectorXd actual_pos = Eigen::VectorXd::Zero(K_);
  for (int k = 0; k < K_; ++k) {
    Eigen::VectorXd d_all(num_fixed_ + num_free_);
    d_all.head(num_fixed_) = d_f_vec[k];
    d_all.tail(num_free_) = d_p_vec[k];

    // Get the coefficients out.
    // L is shorthand for A_inv * M.
    p_vec[k] = L_ * d_all;

    actual_pos(k) = T.transpose() * p_vec[k];
  }

  J_w = (actual_pos - position).norm();

  // Fill in gradients too.
  if (gradients != nullptr) {
    gradients->resize(K_);
    for (int k = 0; k < K_; ++k) {
      (*gradients)[k].resize(num_free_);
      (*gradients)[k].setZero();
      Eigen::MatrixXd df_dpk(K_, num_segments * N);
      df_dpk.setZero();
      df_dpk.row(k) = T;
      (*gradients)[k] =
          ((actual_pos - position) / J_w).transpose() * df_dpk * L_pp;
    }
  }

  return J_w;
}

template <int N>
void Loco<N>::getTVector(double t, Eigen::VectorXd* T) const {
  T->resize(N);
  for (int i = 0; i < N; ++i) {
    (*T)(i) = pow(t, i);
  }
}

template <int N>
void Loco<N>::printMatlabSampledTrajectory(double dt) const {
  mav_trajectory_generation::Trajectory trajectory;
  poly_opt_.getTrajectory(&trajectory);

  std::vector<Eigen::VectorXd> result;
  std::vector<double> sampling_times;
  trajectory.evaluateRange(trajectory.getMinTime(), trajectory.getMaxTime(), dt,
                           0, &result, &sampling_times);

  // Layout: [t, x, y, z...]
  Eigen::MatrixXd output(sampling_times.size(), K_ + 1);
  for (size_t i = 0; i < sampling_times.size(); i++) {
    output(i, 0) = sampling_times[i];
    output.row(i).segment(1, K_) = result[i].transpose();
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
  double d = distance - config_.robot_radius;

  if (d < 0) {
    result = -d + 0.5 * config_.epsilon;
  } else if (d <= config_.epsilon) {
    double epsilon_distance = d - config_.epsilon;
    result = 0.5 * 1.0 / config_.epsilon * epsilon_distance * epsilon_distance;
  } else {
    result = 0.0;
  }
  return result;
}

template <int N>
void Loco<N>::potentialGradientFunction(
    double distance, const Eigen::VectorXd& distance_gradient,
    Eigen::VectorXd* gradient_out) const {
  double d = distance - config_.robot_radius;

  if (d < 0) {
    *gradient_out = -distance_gradient;
  } else if (d <= config_.epsilon) {
    *gradient_out =
        1.0 / config_.epsilon *
        (d * distance_gradient - config_.epsilon * distance_gradient);
  } else {
    gradient_out->setZero(K_);
  }
}

template <int N>
double Loco<N>::computePotentialCostAndGradient(
    const Eigen::VectorXd& position, Eigen::VectorXd* gradient) const {
  Eigen::VectorXd distance_gradient(K_);
  distance_gradient.setZero();
  Eigen::VectorXd increment(K_);

  double d;
  if (gradient != nullptr) {
    d = distance_and_gradient_function_(position, &distance_gradient);
  } else {
    d = distance_and_gradient_function_(position, nullptr);
  }

  double c = potentialFunction(d);
  if (gradient != nullptr) {
    potentialGradientFunction(d, distance_gradient, gradient);
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
  if (gradient != nullptr) {
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
double Loco<N>::getCost() const {
  return computeTotalCostAndGradients(nullptr);
}

template <int N>
double Loco<N>::getNumericalDistanceAndGradient(const Eigen::VectorXd& position,
                                                Eigen::VectorXd* gradient) {
  gradient->setZero(K_);
  Eigen::VectorXd increment(K_);
  double d = distance_function_(position);

  if (gradient != nullptr) {
    for (int k = 0; k < K_; ++k) {
      // Gradient computations for a single dimension.
      increment.setZero();
      increment(k) = config_.map_resolution;
      double left_distance = distance_function_(position - increment);
      double right_distance = distance_function_(position + increment);
      double dim_gradient =
          (right_distance - left_distance) / (2.0 * config_.map_resolution);
      (*gradient)(k) = dim_gradient;
    }
    return d;
  }
}

}  // namespace loco_planner

#endif  // LOCO_PLANNER_IMPL_LOCO_IMPL_H_
