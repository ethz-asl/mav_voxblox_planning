cmake_minimum_required(VERSION 2.8.3)
project(voxblox_skeleton_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/skeleton_global_planner.cpp
  src/skeleton_graph_planner.cpp
)

############
# BINARIES #
############
cs_add_executable(skeleton_global_planner_node
  src/skeleton_global_planner_node.cpp
)
target_link_libraries(skeleton_global_planner_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
