#ifndef MAV_PLANNING_VOXBLOX_VOXBLOX_PLANNER_H
#define MAV_PLANNING_VOXBLOX_VOXBLOX_PLANNER_H

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_planning_voxblox/map_interface.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>

namespace mav_planning {

class VoxbloxPlanner : public MapInterface {
public:
  VoxbloxPlanner(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private);

  virtual ~VoxbloxPlanner() {};

  virtual void initializeMap();

  void visualizeMap();

  // map functions
  bool isMapInitialized();

  double getMapDistance(const Eigen::Vector3d &position) const;
  double getMapWeight(const Eigen::Vector3d& position) const;

  bool checkCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
      const double& robot_radius) const;

  void computeMapBounds(Eigen::Vector3d *lower_bound,
                        Eigen::Vector3d *upper_bound);

  void setVerbose(bool /*verbose*/) {};

protected:
  // Map!
  voxblox::EsdfServer voxblox_server_;
  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;

  std::string input_filepath_;
  bool visualize_;
};

}

#endif // MAV_PLANNING_VOXBLOX_VOXBLOX_PLANNER_H
