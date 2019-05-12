cmake_minimum_required(VERSION 2.8.3)
project(voxblox_planning_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/path_shortening.cpp
  src/gain_evaluator.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
