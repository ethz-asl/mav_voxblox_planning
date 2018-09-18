#include <voxblox/core/block.h>
#include <voxblox/core/voxel.h>

#include "voxblox_skeleton/skeleton_voxel.h"

namespace voxblox {

template <>
void Block<SkeletonVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];
    const int32_t bytes_3 = data[data_idx + 2u];
    SkeletonVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.distance), &bytes_1, sizeof(bytes_1));

    voxel.num_basis_points = static_cast<uint8_t>(bytes_2 & 0x000000FF);
    voxel.is_face = static_cast<bool>(bytes_2 & 0x0000FF00);
    voxel.is_edge = static_cast<bool>(bytes_2 & 0x00FF0000);
    voxel.is_vertex = static_cast<bool>(bytes_2 & 0xFF000000);

    voxel.vertex_id = static_cast<int64_t>(bytes_3);
  }
}

template <>
void Block<SkeletonVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const SkeletonVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.distance);
    data->push_back(*bytes_1_ptr);
    // REPACK!!!
    uint8_t byte1 = voxel.num_basis_points;
    uint8_t byte2 = voxel.is_face;
    uint8_t byte3 = voxel.is_edge;
    uint8_t byte4 = voxel.is_vertex;
    data->push_back(static_cast<uint32_t>(byte1) |
                    (static_cast<uint32_t>(byte2) << 8) |
                    (static_cast<uint32_t>(byte3) << 16) |
                    (static_cast<uint32_t>(byte4) << 24));
    // ...Just compress vertexID down to a 32-bit integer. Sure this will
    // never cause any problems down the line.
    int32_t bytes_3 = -1;
    if (voxel.vertex_id > 0) {
      bytes_3 = static_cast<int32_t>(voxel.vertex_id);
    }
    data->push_back(bytes_3);
  }

  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void mergeVoxelAIntoVoxelB(const SkeletonVoxel& voxel_A,
                           SkeletonVoxel* voxel_B) {
  // This makes absolutely no sense.
  // Just set voxel_B = voxel_A.
  *voxel_B = voxel_A;
}

}  // namespace voxblox
