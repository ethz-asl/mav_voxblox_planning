cmake_minimum_required(VERSION 2.8.3)
project(voxblox_rrt_planner)

find_package(catkin_simple REQUIRED)
find_package(ompl)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

#############
# LIBRARIES #
#############
include_directories(${OMPL_INCLUDE_DIRS})
cs_add_library(${PROJECT_NAME}
  src/ompl_rrt_voxblox.cpp
  src/rrt_planner_voxblox.cpp
  src/rrt_planner.cpp
  src/ompl_rrt.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(voxblox_rrt_planner_node
  src/voxblox_rrt_planner_node.cpp
)
target_link_libraries(voxblox_rrt_planner_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})
