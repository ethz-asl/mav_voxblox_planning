cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/color_utils.cpp
  src/path_visualization.cpp
  src/yaw_policy.cpp
  src/visibility_resampling.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
