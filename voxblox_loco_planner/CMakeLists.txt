cmake_minimum_required(VERSION 2.8.3)
project(voxblox_loco_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/voxblox_loco_planner.cpp
)

############
# BINARIES #
############
#cs_add_executable(global_planning_benchmark_node
#  src/global_planning_benchmark_node.cpp
#)
#target_link_libraries(global_planning_benchmark_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
