#include "voxblox_skeleton/skeleton.h"

namespace voxblox {

Skeleton::Skeleton() {}

void Skeleton::getPointcloud(Pointcloud* pointcloud) const {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  pointcloud->reserve(edges_.size());

  for (const SkeletonPoint& point : edges_) {
    pointcloud->push_back(point.point);
  }
}

void Skeleton::getPointcloudWithDistances(Pointcloud* pointcloud,
                                          std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(points_.size());
  distances->reserve(points_.size());

  for (const SkeletonPoint& point : points_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getEdgePointcloudWithDistances(
    Pointcloud* pointcloud, std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(edges_.size());
  distances->reserve(edges_.size());

  for (const SkeletonPoint& point : edges_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getVertexPointcloudWithDistances(
    Pointcloud* pointcloud, std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(vertices_.size());
  distances->reserve(vertices_.size());

  for (const SkeletonPoint& point : vertices_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

}  // namespace voxblox
