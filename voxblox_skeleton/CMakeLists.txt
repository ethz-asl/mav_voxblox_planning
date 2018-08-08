cmake_minimum_required(VERSION 2.8.3)
project(skeleton_voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/skeleton.cpp
  src/skeleton_generator.cpp
  src/skeleton_planner.cpp
  src/skeleton_serialization.cpp
  src/sparse_graph_planner.cpp
  src/voxel_template_matcher.cpp
)
target_link_libraries(${PROJECT_NAME})

############
# BINARIES #
############
cs_add_executable(skeletonizer
  src/skeletonizer.cpp
)
target_link_libraries(skeletonizer ${PROJECT_NAME})

cs_add_executable(skeleton_eval
  src/skeleton_eval.cpp
)
target_link_libraries(skeleton_eval ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
