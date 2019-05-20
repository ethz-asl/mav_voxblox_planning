cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_benchmark)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/global_planning_benchmark.cpp
  src/local_planning_benchmark.cpp
)

############
# BINARIES #
############
cs_add_executable(global_planning_benchmark_node
  src/global_planning_benchmark_node.cpp
)
target_link_libraries(global_planning_benchmark_node ${PROJECT_NAME})

cs_add_executable(local_planning_benchmark_node
  src/local_planning_benchmark_node.cpp
)
target_link_libraries(local_planning_benchmark_node ${PROJECT_NAME})

cs_add_executable(trajectory_recolor
  src/trajectory_recolor.cpp
)
target_link_libraries(trajectory_recolor ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
