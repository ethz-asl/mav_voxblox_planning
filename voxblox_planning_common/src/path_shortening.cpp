#include "voxblox_planning_common/path_shortening.h"

namespace mav_planning {

EsdfPathShortener::EsdfPathShortener() : voxel_size_(0.0) {}

bool EsdfPathShortener::shortenPath(
    const mav_msgs::EigenTrajectoryPointVector& path,
    mav_msgs::EigenTrajectoryPointVector* shortened_path) const {
  CHECK_NOTNULL(shortened_path);
  // Convert it to a list...
  EigenTrajectoryPointList path_list(path.begin(), path.end());

  // Do one round of path shorterning.
  int i = 0;
  constexpr int kMaxShortens = 10;

  bool any_success = false;

  for (int i = 0; i < kMaxShortens; i++) {
    bool success =
        shortenPathList(path_list.begin(), path_list.end(), &path_list);
    any_success |= success;
    if (!success) {
      break;
    }
  }

  shortened_path->assign(path_list.begin(), path_list.end());
  return any_success;
}

// Shortens the path inside the iterator, leaving start and end the same.
bool EsdfPathShortener::shortenPathList(
    const EigenTrajectoryPointList::iterator& start_iter,
    const EigenTrajectoryPointList::iterator& end_iter,
    EigenTrajectoryPointList* path_list) const {
  if (start_iter == end_iter) {
    return false;
  }

  EigenTrajectoryPointList::iterator iter = start_iter;
  EigenTrajectoryPointList::iterator last_iter = end_iter;
  last_iter--;

  // If we can shortcut this...
  if (!isLineInCollision(start_iter->position_W, last_iter->position_W)) {
    // Then we remove everything between start iter and end iter.
    iter++;
    int i = 0;
    while (iter != last_iter && iter != end_iter) {
      iter = path_list->erase(iter);
      i++;
    }
    return i > 0;
  } else {
    // Recursively split the remaining path list into left and right half.
    EigenTrajectoryPointList::iterator middle_iter = start_iter;
    for (int i = 0; iter != last_iter; ++iter, ++i) {
      if (i % 2 == 0) {
        middle_iter++;
      }
    }

    if (middle_iter == start_iter || middle_iter == end_iter) {
      return false;
    }

    bool left_success = shortenPathList(start_iter, middle_iter, path_list);
    bool right_success = shortenPathList(middle_iter, end_iter, path_list);
    return left_success || right_success;
  }
  return false;
}

bool EsdfPathShortener::isLineInCollision(const Eigen::Vector3d& start,
                                          const Eigen::Vector3d& end) const {
  CHECK_NOTNULL(esdf_layer_);
  CHECK_GT(voxel_size_, 0.0);

  Eigen::Vector3d direction = (end - start);
  double distance = direction.norm();
  direction.normalize();

  // Don't check anything smaller than 1 voxel.
  if (distance < voxel_size_) {
    return false;
  }

  // Start at the start, keep going by distance increments...
  Eigen::Vector3d current_position = start;
  double distance_so_far = 0.0;

  while (distance_so_far <= distance) {
    voxblox::EsdfVoxel* esdf_voxel = esdf_layer_->getVoxelPtrByCoordinates(
        current_position.cast<voxblox::FloatingPoint>());
    if (esdf_voxel == nullptr) {
      return true;
    }
    if (esdf_voxel->distance < constraints_.robot_radius) {
      return true;
    }

    double step_size =
        std::max(voxel_size_, esdf_voxel->distance - constraints_.robot_radius);

    current_position += direction * step_size;
    distance_so_far += step_size;
  }
  return false;
}

}  // namespace mav_planning
