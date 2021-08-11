cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/voxblox_planner.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
