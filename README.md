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

**voxblox**
      <p>Helen Oleynikova, Zachary Taylor, Marius Fehr, Roland Siegwart, and Juan Nieto, “<b>Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning</b>”. In <i>IEEE Int. Conf. on Intelligent Robots and Systems (IROS)</i>, October 2017.<br>
        [<a href="publications/iros_2017_voxblox.pdf">pdf</a> | <a href="publications/iros_2017_voxblox_bibtex.txt">bibtex</a> | <a href="https://www.youtube.com/watch?v=ZGvnGFnTVR8">video</a> | <a href="https://arxiv.org/abs/1611.03631">arxiv</a> ]
      </p>

**loco planning**
      <p>Helen Oleynikova, Michael Burri, Zachary Taylor, Juan Nieto, Roland Siegwart, and Enric Galceran, “<b>Continuous-Time Trajectory Optimization for Online UAV Replanning</b>”. In <i>IEEE Int. Conf. on Intelligent Robots and Systems (IROS)</i>, October 2016.<br>
        [<a href="publications/iros_2016_replanning.pdf">pdf</a> | <a href="publications/iros_2016_replanning_bibtex.txt">bibtex</a> | <a href="https://www.youtube.com/watch?v=-cm-HkTI8vw">video</a>]
      </p>

**loco planning with voxblox**
      <p>Helen Oleynikova, Zachary Taylor, Roland Siegwart, and Juan Nieto, “<b>Safe Local Exploration for Replanning in Cluttered Unknown Environments for Micro-Aerial Vehicles</b>”. <i>IEEE Robotics and Automation Letters</i>, 2018.<br>
        [<a href="publications/ral_2018_local_exploration.pdf">pdf</a> | <a href="publications/ral_2018_bibtex.txt">bibtex</a> | <a href="https://www.youtube.com/watch?v=rAJwD2kr7c0">video</a> | <a href="https://arxiv.org/abs/1710.00604">arxiv</a> ]
      </p>

**voxblox skeleton and skeleton planning**
      <p>Helen Oleynikova, Zachary Taylor, Roland Siegwart, and Juan Nieto, “<b>Sparse 3D Topological Graphs for Micro-Aerial Vehicle Planning</b>”. In <i>IEEE Int. Conf. on Intelligent Robots and Systems (IROS)</i>, October 2018.<br>
        [<a href="publications/iros_2018_skeleton.pdf">pdf</a> | <a href="publications/iros_2018_skeleton_bibtex.txt">bibtex</a> | <a href="https://www.youtube.com/watch?v=U_6rk-SF0Nw">video</a> | <a href="https://arxiv.org/abs/1803.04345">arxiv</a> ]
      </p>

# Getting Started
## Installation
This package is intended to be used with Ubuntu 16.04 and [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or above.
After installing ROS, install some extra dependencies, substituting kinetic for your ROS version as necessary:
```
sudo apt-get install ros-kinetic-cmake-modules python-wstool python-catkin-tools libyaml-cpp-dev protobuf-compiler autoconf
```
Then if not already done so, set up a new catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```
If using [**SSH keys for github**](https://help.github.com/articles/connecting-to-github-with-ssh/) (recommended):
```
cd ~/catkin_ws/src/
git clone git@github.com:ethz-asl/mav_voxblox_planning.git
wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
wstool update
```

If **not using SSH** keys but using https instead:
```
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/mav_voxblox_planning.git
wstool init . ./mav_voxblox_planning/install/install_https.rosinstall
wstool update
```

If you have already initalized wstool replace the above `wstool init` with `wstool merge -t`

Compile:
```
cd ~/catkin_ws/src/
catkin build mav_voxblox_planning
```

## Download maps
We've prepared a number of maps for you to try out our planning on.
The archive is 260 MB big and available [**here**](http://robotics.ethz.ch/~asl-datasets/2018_mav_voxblox_planning/mav_voxblox_planning_maps.zip).

It contains 6 maps from 3 scenarios: machine hall (indoor), shed (mixed indoor and outdoor area), and rubble (outdoor), each with Intel Realsense D400-series scans and with grayscale stereo matching scans. Each map features 3 files: esdf, skeleton (esdf + skeleton diagram), and sparse graph, which contains just the sparse graph generated using skeletonization.



| Dataset | Realsense | Stereo |
| ---- | --------- | --------- |
| Machine Hall | ![machine_hall_rs](https://user-images.githubusercontent.com/5616392/46145669-344edd00-c261-11e8-8722-f12e4157f877.png)  | ![machine_hall_stereo](https://user-images.githubusercontent.com/5616392/46145670-344edd00-c261-11e8-8a07-d4b710daf78f.png)  |
| Rubble | ![rubble_rs](https://user-images.githubusercontent.com/5616392/46145671-344edd00-c261-11e8-8c3c-6460e458e3d0.png) | ![rubble_stereo](https://user-images.githubusercontent.com/5616392/46145672-344edd00-c261-11e8-9e3e-3e349f95bcf9.png)   |
| Shed | ![shed_rs](https://user-images.githubusercontent.com/5616392/46145673-344edd00-c261-11e8-8002-0369d728d02d.png) | ![shed_stereo](https://user-images.githubusercontent.com/5616392/46145674-344edd00-c261-11e8-91bc-3cd042604bae.png) |




## Try out RRT and Skeleton planning


# Advanced
## Skeletonize your own maps



