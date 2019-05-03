#ifndef VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/utils.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/camera_model.h>

namespace mav_planning {

class GainEvaluator {
 public:
  GainEvaluator();

  // Functions to set up the internal camera model.
  void setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                   double min_distance, double max_distance);
  void setCameraModelParametersFocalLength(const Eigen::Vector2d& resolution,
                                           double focal_length,
                                           double min_distance,
                                           double max_distance);
  void setCameraExtrinsics(const voxblox::Transformation& T_C_B);

  // Bind the TSDF layer to one OWNED BY ANOTHER OBJECT. It is up to the user
  // to ensure the layer exists and does not go out of scope.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);

  // Evaluate the exploration gain by counting unknown voxels in frustum.
  // Modulus is how much to subsample the queried view frustum. Modulus = 1:
  // evaluate the full frustum.
  double evaluateExplorationGainVoxelCount(
      const mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use raycasting to discard occluded voxels.
  double evaluateExplorationGainWithRaycasting(
      const mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Also use raycasting to discard occluded voxels, Bircher-style
  // implementation.
  double evaluateExplorationGainBircher(
      const mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
  // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::CameraModel cam_model_;

  // Cached parameters of the layer.
  float voxel_size_;
  float voxel_size_inv_;
  int voxels_per_side_;
  float voxels_per_side_inv_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_
