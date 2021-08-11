cmake_minimum_required(VERSION 2.8.3)
project(voxblox_loco_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/voxblox_loco_planner.cpp
  src/shotgun_planner.cpp
  src/goal_point_selector.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
