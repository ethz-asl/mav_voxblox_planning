#ifndef VOXBLOX_SKELETON_NANOFLANN_INTERFACE_H_
#define VOXBLOX_SKELETON_NANOFLANN_INTERFACE_H_

#include "voxblox_skeleton/nanoflann/nanoflann.h"
#include "voxblox_skeleton/skeleton.h"

namespace voxblox {

// Adaptor is British spelling of Adapter...
class SkeletonPointVectorAdapter {
 public:
  SkeletonPointVectorAdapter(const AlignedVector<SkeletonPoint>& points)
      : points_(points) {}

  inline const AlignedVector<SkeletonPoint>& derived() const { return points_; }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class.
  inline FloatingPoint kdtree_get_pt(const size_t idx, int dim) const {
    return points_[idx].point(dim);
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

 private:
  const AlignedVector<SkeletonPoint>& points_;
};

class SkeletonVertexMapAdapter {
 public:
  SkeletonVertexMapAdapter(const std::map<int64_t, SkeletonVertex>& vertices)
      : vertices_(vertices) {}

  inline const std::map<int64_t, SkeletonVertex>& derived() const {
    return vertices_;
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return vertices_.size(); }

  // Returns the dim'th component of the idx'th point in the class.
  // Sooooooo actually mapped to the order of the map, not the real vertex
  // index. This could definitely have problems.
  inline FloatingPoint kdtree_get_pt(const size_t idx, int dim) const {
    auto iter = vertices_.begin();
    for (size_t i = 0; i < idx; ++i) {
      iter++;
    }
    return iter->second.point(dim);
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

 private:
  const std::map<int64_t, SkeletonVertex>& vertices_;
};

class DirectSkeletonVertexMapAdapter {
 public:
  DirectSkeletonVertexMapAdapter(
      const std::map<int64_t, SkeletonVertex>& vertices)
      : vertices_(vertices) {}

  inline const std::map<int64_t, SkeletonVertex>& derived() const {
    return vertices_;
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const {
    if (vertices_.empty()) {
      return 0;
    }
    auto iter = vertices_.end();
    iter--;
    return iter->first + 1;
  }

  inline FloatingPoint kdtree_get_pt(const size_t idx, int dim) const {
    auto iter = vertices_.find(idx);
    if (iter != vertices_.end()) {
      return iter->second.point(dim);
    }
    return std::numeric_limits<FloatingPoint>::max();
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

 private:
  const std::map<int64_t, SkeletonVertex>& vertices_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_NANOFLANN_INTERFACE_H_
