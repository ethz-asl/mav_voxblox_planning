#ifndef MAV_PLANNING_VOXBLOX_MAP_INTERFACE_H
#define MAV_PLANNING_VOXBLOX_MAP_INTERFACE_H

#include <Eigen/Core>

namespace mav_planning {

  class MapInterface {
   public:
    typedef std::shared_ptr<MapInterface> Ptr;
    typedef std::shared_ptr<const MapInterface> ConstPtr;

    // constructor
    MapInterface() {};
    virtual ~MapInterface() {};

    // functions
    virtual void initializeMap() = 0;
    virtual void visualizeMap() = 0;
    virtual bool isMapInitialized() = 0;
    virtual void computeMapBounds(Eigen::Vector3d* lower_bound,
                                  Eigen::Vector3d* upper_bound) const = 0;

    // accessors
    virtual double getMapDistance(const Eigen::Vector3d &position) const = 0;
    virtual double getMapWeight(const Eigen::Vector3d& position) const = 0;
    double getVoxelSize() const {return voxel_size_;};

  protected:
    double voxel_size_;  // Cache the size of the voxels used by the map.
  };

} // namespace cblox

#endif // MAV_PLANNING_VOXBLOX_MAP_INTERFACE_H
