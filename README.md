# mav_voxblox_planning
MAV planning tools using voxblox as the map representation.

**NOTE: THIS PACKAGE IS UNDER ACTIVE DEVELOPMENT! Things are subject to change at any time.**

## Contents and Future Work
### Included
* Global Planning
  * RRT*, RRT Connect, BIT*, PRM... (OMPL interface) `voxblox_rrt_planner`
  * Planning on skeleton sparse graphs `voxblox_skeleton_planner`
  * Skeleton sparse graphs (topology) generation from ESDF voxblox maps `voxblox_skeleton`
  * Randomized benchmark on a variety of real maps `mav_planning_benchmark`
* Path Smoothing (between waypoints)
  * Velocity ramp `mav_path_smoothing`
  * Polynomial with adding additional vertices at collisions `mav_path_smoothing`
  * Local polynomial optimization (Loco), unconstraining waypoints and minimizing collisions `mav_path_smoothing, loco_planner`
* RVIZ Planning Plugin
  * Allows dragging around start and goal markers, sending planning requests to global planners `mav_planning_rviz`

### To Come (by end of October 2018)
* Local Planning
  * Collision avoidance in unknown environments
  * Local planning benchmark
* RVIZ Planning Plugin
  * Support for entering multiple waypoints

# Papers and References
If using these, please cite:



# Getting Started



