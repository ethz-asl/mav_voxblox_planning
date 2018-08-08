cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner_voxblox)

find_package(catkin_simple REQUIRED)
find_package(OMPL REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/ompl_rrt_voxblox.cpp
  src/rrt_planner_voxblox.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(rrt_planner_voxblox_node
  src/rrt_planner_voxblox_node.cpp
)
target_link_libraries(rrt_planner_voxblox_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
