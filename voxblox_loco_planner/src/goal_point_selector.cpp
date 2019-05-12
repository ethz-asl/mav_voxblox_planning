#include "voxblox_loco_planner/goal_point_selector.h"

namespace mav_planning {

GoalPointSelector::GoalPointSelector() {}

void GoalPointSelector::setParameters(
    const GoalPointSelectorParameters& params) {
  params_ = params;
}

GoalPointSelectorParameters GoalPointSelector::getParameters() const {
  return params_;
}

void GoalPointSelector::setParametersFromRos(const ros::NodeHandle& nh) {
  std::string strategy = "none";

  nh.param("goal_selector_strategy", strategy, strategy);
  if (strategy == "none") {
    params_.strategy = GoalPointSelectorParameters::kNoIntermediateGoal;
  } else if (strategy == "random") {
    params_.strategy = GoalPointSelectorParameters::kRandom;
  } else if (strategy == "local" || strategy == "local_exploration") {
    params_.strategy = GoalPointSelectorParameters::kLocalExploration;
  } else {
    ROS_ERROR_STREAM("[Goal Point Selector] Invalid strategy: " << strategy);
  }

  nh.param("goal_selector_range", params_.random_sample_range,
           params_.random_sample_range);
}

void GoalPointSelector::setTsdfMap(
    const std::shared_ptr<voxblox::TsdfMap>& tsdf_map) {
  tsdf_map_ = tsdf_map;
  if (tsdf_map) {
    gain_evaluator_.setTsdfLayer(tsdf_map->getTsdfLayerPtr());
  }
}

bool GoalPointSelector::selectNextGoal(
    const mav_msgs::EigenTrajectoryPoint& global_goal,
    const mav_msgs::EigenTrajectoryPoint& current_goal,
    const mav_msgs::EigenTrajectoryPoint& current_pose,
    mav_msgs::EigenTrajectoryPoint* next_goal) {
  if (params_.strategy == GoalPointSelectorParameters::kNoIntermediateGoal) {
    *next_goal = global_goal;
    return false;
  }

  const double kCloseEnough = 0.1;  // meters.

  // Regardless of what we're doing... If the current goal isn't the global
  // goal, switch to the global goal.
  if ((current_goal.position_W - global_goal.position_W).norm() >
      kCloseEnough) {
    *next_goal = global_goal;
    return true;
  }

  // Otherwise we select a goal using whatever strategy specified.
  if (params_.strategy == GoalPointSelectorParameters::kRandom) {
    selectRandomPose(current_pose, params_.random_sample_range, next_goal);
    return true;
  }

  if (params_.strategy == GoalPointSelectorParameters::kLocalExploration) {
    if (!tsdf_map_) {
      return false;
    }
    return selectLocalExplorationGoal(global_goal, current_goal, current_pose,
                                      next_goal);
  }
}

void GoalPointSelector::selectRandomPose(
    const mav_msgs::EigenTrajectoryPoint& input_pose, double range_meters,
    mav_msgs::EigenTrajectoryPoint* sampled_pose) const {
  Eigen::Vector3d random_position;
  // Uniformly sample theta and phi angles.
  // Uniformly sample radius between 0 and r.
  double theta = randMToN(0, M_PI * 2.0);
  double phi = randMToN(-M_PI / 2.0, M_PI / 2.0);
  double r = randMToN(0.0, range_meters);

  random_position.x() = r * cos(theta) * cos(phi);
  random_position.y() = r * sin(phi);
  random_position.z() = r * sin(theta) * cos(phi);

  // Also randomly sample the yaw.
  double yaw = randMToN(-M_PI, M_PI);
  sampled_pose->position_W = input_pose.position_W + random_position;
  sampled_pose->setFromYaw(yaw);
}

bool GoalPointSelector::selectRandomFreePose(
    const mav_msgs::EigenTrajectoryPoint& input_pose, double range_meters,
    mav_msgs::EigenTrajectoryPoint* sampled_pose) const {
  if (!tsdf_map_) {
    return false;
  }
  // Get a pointer to the layer to use later.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_->getTsdfLayerPtr();
  CHECK_NOTNULL(tsdf_layer);

  for (int i = 0; i < params_.max_random_tries; ++i) {
    selectRandomPose(input_pose, range_meters, sampled_pose);

    // Look up the voxel in the TSDF.
    voxblox::BlockIndex block_index =
        tsdf_layer->computeBlockIndexFromCoordinates(
            sampled_pose->position_W.cast<float>());

    const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
        tsdf_layer->getBlockPtrByIndex(block_index);
    if (block_ptr) {
      const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(
          sampled_pose->position_W.cast<float>());
      // Just return with this pose if it's in free space.
      if (voxel.weight >= 1e-6 && voxel.distance > 0.0) {
        return true;
      }
    }
  }
  return false;
}

bool GoalPointSelector::selectLocalExplorationGoal(
    const mav_msgs::EigenTrajectoryPoint& global_goal,
    const mav_msgs::EigenTrajectoryPoint& current_goal,
    const mav_msgs::EigenTrajectoryPoint& current_pose,
    mav_msgs::EigenTrajectoryPoint* next_goal) {
  const double max_goal_dist =
      (global_goal.position_W - current_pose.position_W).norm() +
      params_.random_sample_range;

  double best_gain = 0.0;
  mav_msgs::EigenTrajectoryPoint best_point;

  // First, select N random points within a radius of the current pose.
  for (int i = 0; i < params_.num_exploration_samples; i++) {
    mav_msgs::EigenTrajectoryPoint sampled_pose;
    selectRandomFreePose(current_pose, params_.random_sample_range,
                         &sampled_pose);

    // For every point, evaluate its total gain: sum of distance of final
    // point to the goal, and the exploration gain from the 5% downsampled
    // amount.
    double exploration_gain = evaluateExplorationGain(sampled_pose);

    Eigen::Vector3d travel_ray =
        sampled_pose.position_W - current_pose.position_W;
    double yaw = atan2(travel_ray.y(), travel_ray.x());
    sampled_pose.setFromYaw(yaw);

    // Normalize the goal gain by the exploration range of the algorithm to
    // make sure the scoring is consistent across different settings.
    double goal_gain =
        (max_goal_dist -
         (global_goal.position_W - sampled_pose.position_W).norm()) /
        max_goal_dist;

    // Use heuristics to weigh between the two factors.
    double total_gain =
        params_.w_exploration * exploration_gain + params_.w_goal * goal_gain;
    if (total_gain >= best_gain) {
      best_gain = total_gain;
      best_point = sampled_pose;
    }
  }

  *next_goal = best_point;
  return true;
}

double GoalPointSelector::evaluateExplorationGain(
    const mav_msgs::EigenTrajectoryPoint& pose) {
  return gain_evaluator_.evaluateExplorationGainVoxelCount(pose,
                                                           params_.exp_modulus);
}

}  // namespace mav_planning
