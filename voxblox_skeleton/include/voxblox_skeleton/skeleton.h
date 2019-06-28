#ifndef VOXBLOX_SKELETON_SKELETON_H_
#define VOXBLOX_SKELETON_SKELETON_H_

#include <map>

#include <voxblox/core/common.h>

#include "voxblox_skeleton/skeleton_voxel.h"

namespace voxblox {

struct SkeletonPoint {
  Point point = Point::Zero();
  float distance = 0.0f;
  int num_basis_points = 0;
  AlignedVector<Point> basis_directions;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Skeleton {
 public:
  Skeleton();

  // Access to all the skeleton points.
  const AlignedVector<SkeletonPoint>& getSkeletonPoints() const {
    return points_;
  }
  const AlignedList<SkeletonPoint>& getEdgePoints() const { return edges_; }
  const AlignedVector<SkeletonPoint>& getVertexPoints() const {
    return vertices_;
  }

  AlignedVector<SkeletonPoint>& getSkeletonPoints() { return points_; }
  AlignedList<SkeletonPoint>& getEdgePoints() { return edges_; }
  AlignedVector<SkeletonPoint>& getVertexPoints() { return vertices_; }

  // Converts the points to a pointcloud with no other information, for all
  // points on the GVD.
  void getPointcloud(Pointcloud* pointcloud) const;

  // Also get a vector for the distance information.
  void getPointcloudWithDistances(Pointcloud* pointcloud,
                                  std::vector<float>* distances) const;
  void getEdgePointcloudWithDistances(Pointcloud* pointcloud,
                                      std::vector<float>* distances) const;
  void getVertexPointcloudWithDistances(Pointcloud* pointcloud,
                                        std::vector<float>* distances) const;

 private:
  AlignedVector<SkeletonPoint> points_;

  // Subsets of points (just copies because we don't care)
  AlignedVector<SkeletonPoint> vertices_;
  AlignedList<SkeletonPoint> edges_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_SKELETON_H_
