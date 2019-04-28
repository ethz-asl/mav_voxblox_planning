#ifndef VOXBLOX_PLANNING_COMMON_PATH_SHORTENING_H_
#define VOXBLOX_PLANNING_COMMON_PATH_SHORTENING_H_

#include <list>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/physical_constraints.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

namespace mav_planning {

class EsdfPathShortener {
 public:
  typedef std::list<mav_msgs::EigenTrajectoryPoint,
                    Eigen::aligned_allocator<mav_msgs::EigenTrajectoryPoint>>
      EigenTrajectoryPointList;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EsdfPathShortener();

  void setConstraints(const PhysicalConstraints& constraints) {
    constraints_ = constraints;
  }
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer) {
    esdf_layer_ = esdf_layer;
    if (esdf_layer_) {
      voxel_size_ = esdf_layer_->voxel_size();
    }
  }

  bool shortenPath(const mav_msgs::EigenTrajectoryPointVector& path,
                   mav_msgs::EigenTrajectoryPointVector* shortened_path) const;

  bool shortenPathList(const EigenTrajectoryPointList::iterator& start_iter,
                       const EigenTrajectoryPointList::iterator& end_iter,
                       EigenTrajectoryPointList* path_list) const;

  bool isLineInCollision(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end) const;

 private:
  PhysicalConstraints constraints_;

  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;

  // Cache the voxel size, as a double.
  double voxel_size_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_PLANNING_COMMON_PATH_SHORTENING_H_
