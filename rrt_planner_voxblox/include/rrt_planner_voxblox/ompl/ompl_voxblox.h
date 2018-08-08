#ifndef RRT_PLANNER_VOXBLOX_OMPL_OMPL_VOXBLOX_H_
#define RRT_PLANNER_VOXBLOX_OMPL_OMPL_VOXBLOX_H_

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>

#include "rrt_planner_voxblox/ompl/ompl_types.h"


namespace ompl {
namespace mav {

class TsdfVoxbloxValidityChecker : public base::StateValidityChecker {
 public:
  typedef voxblox::AnyIndexHashMapType<voxblox::VoxelIndexList>::type
      BlockVoxelListMap;

  TsdfVoxbloxValidityChecker(const base::SpaceInformationPtr& space_info,
                             voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer,
                             double robot_radius)
      : base::StateValidityChecker(space_info),
        tsdf_layer_(tsdf_layer),
        robot_radius_(robot_radius) {
    CHECK_NOTNULL(tsdf_layer_);
    tsdf_voxel_size_ = tsdf_layer_->voxel_size();
  }

  virtual bool isValid(const base::State* state) const {
    Eigen::Vector3d robot_position = omplToEigen(state);
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    bool collision = checkCollisionWithRobot(robot_position);
    // We check the VALIDITY of the state, and the function above returns
    // whether the state was in COLLISION.
    return !collision;
  }

  inline bool checkCollisionWithRobot(
      const Eigen::Vector3d& robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

    BlockVoxelListMap block_voxel_list;
    getSphereAroundPoint(robot_point, robot_radius_, &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
         block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          tsdf_layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex& voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          continue;
        }
        const voxblox::TsdfVoxel& tsdf_voxel =
            block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.weight < voxblox::kEpsilon) {
          continue;
        }
        if (tsdf_voxel.distance <= 0.0f) {
          return true;
        }
      }
    }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
    return false;
  }

  inline void getSphereAroundPoint(const voxblox::Point& center,
                                   voxblox::FloatingPoint radius,
                                   BlockVoxelListMap* block_voxel_list) const {
    CHECK_NOTNULL(block_voxel_list);
    block_voxel_list->clear();
    // search a cube with side length 2*radius
    for (voxblox::FloatingPoint x = -radius; x <= radius;
         x += tsdf_voxel_size_) {
      for (voxblox::FloatingPoint y = -radius; y <= radius;
           y += tsdf_voxel_size_) {
        for (voxblox::FloatingPoint z = -radius; z <= radius;
             z += tsdf_voxel_size_) {
          voxblox::Point point(x, y, z);

          // check if point is inside the spheres radius
          if (point.squaredNorm() <= radius * radius) {
            // convert to global coordinate
            point += center;

            voxblox::BlockIndex block_index =
                tsdf_layer_->computeBlockIndexFromCoordinates(point);

            const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
                tsdf_layer_->getBlockPtrByIndex(block_index);
            if (block_ptr) {
              (*block_voxel_list)[block_index].push_back(
                  block_ptr->computeVoxelIndexFromCoordinates(point));
            }
          }
        }
      }
    }
  }

 protected:
  // We're gonna use the TSDF map because this way we can choose how to treat
  // actually unknown space. This is kind of a massive hack... And slower ;)
  // But it's all OK I guess, this is just for benchmarking.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;

  double robot_radius_;
  double tsdf_voxel_size_;
};

class EsdfVoxbloxValidityChecker : public base::StateValidityChecker {
 public:
  EsdfVoxbloxValidityChecker(const base::SpaceInformationPtr& space_info,
                             voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer,
                             double robot_radius)
      : base::StateValidityChecker(space_info),
        esdf_layer_(esdf_layer),
        interpolator_(esdf_layer),
        robot_radius_(robot_radius) {
    CHECK_NOTNULL(esdf_layer_);
  }

  virtual bool isValid(const base::State* state) const {
    Eigen::Vector3d robot_position = omplToEigen(state);
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    bool collision = checkCollisionWithRobot(robot_position);
    // We check the VALIDITY of the state, and the function above returns
    // whether the state was in COLLISION.
    return !collision;
  }

  inline bool checkCollisionWithRobot(
      const Eigen::Vector3d& robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
    constexpr bool interpolate = true;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(
        robot_position.cast<voxblox::FloatingPoint>(), &distance, interpolate);
    if (!success) {
      return false;
    }

    return robot_radius_ > distance;
  }

 protected:
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;

  // Interpolator for the layer.
  voxblox::Interpolator<voxblox::EsdfVoxel> interpolator_;

  double robot_radius_;
};

}  // namespace mav
}  // namespace ompl

#endif  // RRT_PLANNER_VOXBLOX_OMPL_OMPL_VOXBLOX_H_
