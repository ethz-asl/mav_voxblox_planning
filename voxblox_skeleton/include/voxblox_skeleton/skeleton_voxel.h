#ifndef VOXBLOX_SKELETON_SKELETON_VOXEL_H_
#define VOXBLOX_SKELETON_SKELETON_VOXEL_H_

#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

namespace voxblox {

struct SkeletonVoxel {
  float distance = 0.0f;
  uint8_t num_basis_points = 0u;
  bool is_face = false;
  bool is_edge = false;
  bool is_vertex = false;
  int64_t vertex_id = -1;
};

// Used for serialization only.
namespace voxel_types {
const std::string kSkeleton = "skeleton";
}

template <>
inline std::string getVoxelType<SkeletonVoxel>() {
  return voxel_types::kSkeleton;
}

}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_SKELETON_VOXEL_H_
