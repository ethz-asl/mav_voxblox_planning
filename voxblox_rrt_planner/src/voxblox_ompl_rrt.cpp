#include "voxblox_rrt_planner/voxblox_ompl_rrt.h"

namespace mav_planning {

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      planner_type_(kRrtStar),
      num_seconds_to_plan_(2.5),
      simplify_solution_(true),
      robot_radius_(1.0),
      verbose_(false),
      optimistic_(true),
      trust_approx_solution_(false),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()) {
  nh_private_.param("robot_radius", robot_radius_, robot_radius_);
  nh_private_.param("num_seconds_to_plan", num_seconds_to_plan_,
                    num_seconds_to_plan_);
  nh_private_.param("simplify_solution", simplify_solution_,
                    simplify_solution_);
  nh_private_.param("trust_approx_solution", trust_approx_solution_,
                    trust_approx_solution_);
}

void VoxbloxOmplRrt::setBounds(const Eigen::Vector3d& lower_bound,
                               const Eigen::Vector3d& upper_bound) {
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void VoxbloxOmplRrt::setTsdfLayer(
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  CHECK_NOTNULL(tsdf_layer_);
  voxel_size_ = tsdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setEsdfLayer(
    voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
  esdf_layer_ = esdf_layer;
  CHECK_NOTNULL(esdf_layer_);
  voxel_size_ = esdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setupProblem() {
  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    problem_setup_.setTsdfVoxbloxCollisionChecking(robot_radius_, tsdf_layer_);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_.setEsdfVoxbloxCollisionChecking(robot_radius_, esdf_layer_);
  }
  problem_setup_.setDefaultObjective();
  if (planner_type_ == kRrtConnect) {
    problem_setup_.setRrtConnect();
  } else if (planner_type_ == kRrtStar) {
    problem_setup_.setRrtStar();
  } else if (planner_type_ == kInformedRrtStar) {
    problem_setup_.setInformedRrtStar();
  } else if (planner_type_ == kPrm) {
    problem_setup_.setPrm();
  } else if (planner_type_ == kBitStar) {
    problem_setup_.setBitStar();
  } else {
    problem_setup_.setDefaultPlanner();
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());
    bounds.setLow(2, lower_bound_.z());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());
    bounds.setHigh(2, upper_bound_.z());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::mav::StateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

bool VoxbloxOmplRrt::getPathBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPointVector* solution) {
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  if (problem_setup_.solve(num_seconds_to_plan_)) {
    if (problem_setup_.haveExactSolutionPath()) {
      // Simplify and print.
      // TODO(helenol): look more into this. Appears to actually prefer more
      // vertices with presumably shorter total path length, which is
      // detrimental to polynomial planning.
      if (simplify_solution_) {
        problem_setup_.reduceVertices();
      }
      if (verbose_) {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
    } else {
      ROS_WARN("OMPL planning failed.");
      return false;
    }
  }

  if (problem_setup_.haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), solution);
    return true;
  }
  return false;
}

void VoxbloxOmplRrt::setupFromStartAndGoal(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal) {
  if (planner_type_ == kPrm) {
    std::dynamic_pointer_cast<ompl::geometric::PRM>(problem_setup_.getPlanner())
        ->clearQuery();
  } else {
    problem_setup_.clear();
  }

  ompl::base::ScopedState<ompl::mav::StateSpace> start_ompl(
      problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::mav::StateSpace> goal_ompl(
      problem_setup_.getSpaceInformation());

  start_ompl->values[0] = start.position_W.x();
  start_ompl->values[1] = start.position_W.y();
  start_ompl->values[2] = start.position_W.z();

  goal_ompl->values[0] = goal.position_W.x();
  goal_ompl->values[1] = goal.position_W.y();
  goal_ompl->values[2] = goal.position_W.z();
  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl);
  problem_setup_.setup();
  if (verbose_) {
    problem_setup_.print();
  }
}

void VoxbloxOmplRrt::solutionPathToTrajectoryPoints(
    ompl::geometric::PathGeometric& path,
    mav_msgs::EigenTrajectoryPointVector* trajectory_points) const {
  CHECK_NOTNULL(trajectory_points);
  trajectory_points->clear();
  trajectory_points->reserve(path.getStateCount());

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[0],
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[1],
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[2]);

    mav_msgs::EigenTrajectoryPoint mav_trajectory_point;
    mav_trajectory_point.position_W = mav_position;
    trajectory_points->emplace_back(mav_trajectory_point);
  }
}

bool VoxbloxOmplRrt::getBestPathTowardGoal(
    const mav_msgs::EigenTrajectoryPoint& start,
    const mav_msgs::EigenTrajectoryPoint& goal,
    mav_msgs::EigenTrajectoryPoint::Vector* solution) {
  CHECK_NOTNULL(solution);
  solution->clear();
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  bool solution_found = false;
  solution_found = problem_setup_.solve(num_seconds_to_plan_);
  if (solution_found) {
    if (problem_setup_.haveSolutionPath()) {
      // Simplify and print.
      if (simplify_solution_) {
        problem_setup_.reduceVertices();
      }
      if (verbose_) {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
      solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(),
                                     solution);
      return true;
    }
  }
  // The case where you actually have a solution path has returned by now.
  // Otherwise let's just see what the best we can do is.
  ompl::base::PlannerData planner_data(problem_setup_.getSpaceInformation());
  problem_setup_.getPlanner()->getPlannerData(planner_data);

  // Start traversing the graph and find the node that gets the closest to the
  // actual goal point.
  if (planner_data.numStartVertices() < 1) {
    ROS_ERROR("No start vertices in RRT!");
    return false;
  }

  unsigned int min_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  if (planner_data.numVertices() <= 0) {
    ROS_ERROR("No vertices in RRT!");
    return false;
  }

  // Iterate over all vertices. Check which is the closest.
  for (unsigned int i = 0; i < planner_data.numVertices(); i++) {
    const ompl::base::PlannerDataVertex& vertex = planner_data.getVertex(i);
    double distance =
        getDistanceEigenToState(goal.position_W, vertex.getState());

    if (distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  unsigned int start_index = planner_data.getStartIndex(0);

  // Get the closest vertex back out, and then get its parents.
  mav_msgs::EigenTrajectoryPointVector trajectory_points;

  unsigned int current_index = min_index;
  while (current_index != start_index) {
    // Put this vertex in.
    const ompl::base::PlannerDataVertex& vertex =
        planner_data.getVertex(current_index);

    const ompl::base::State* state_ptr = vertex.getState();
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[0],
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[1],
        state_ptr->as<ompl::mav::StateSpace::StateType>()->values[2]);

    mav_msgs::EigenTrajectoryPoint mav_trajectory_point;
    mav_trajectory_point.position_W = mav_position;
    trajectory_points.emplace_back(mav_trajectory_point);

    std::vector<unsigned int> edges;

    planner_data.getIncomingEdges(current_index, edges);

    if (edges.empty()) {
      break;
    }

    current_index = edges.front();
  }

  // Finally reverse the vector.
  std::reverse(std::begin(trajectory_points), std::end(trajectory_points));

  *solution = trajectory_points;
  return false;
}

double VoxbloxOmplRrt::getDistanceEigenToState(
    const Eigen::Vector3d& eigen, const ompl::base::State* state_ptr) {
  Eigen::Vector3d state_pos(
      state_ptr->as<ompl::mav::StateSpace::StateType>()->values[0],
      state_ptr->as<ompl::mav::StateSpace::StateType>()->values[1],
      state_ptr->as<ompl::mav::StateSpace::StateType>()->values[2]);

  return (eigen - state_pos).norm();
}

}  // namespace mav_planning
