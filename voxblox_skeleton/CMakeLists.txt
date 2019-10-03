cmake_minimum_required(VERSION 2.8.3)
project(voxblox_skeleton)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox_skeleton/skeleton.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

cs_add_library(${PROJECT_NAME}
  src/io/skeleton_io.cpp
  src/skeleton.cpp
  src/skeleton_generator.cpp
  src/skeleton_planner.cpp
  src/skeleton_serialization.cpp
  src/sparse_graph_planner.cpp
  src/voxel_template_matcher.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

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
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS voxblox_skeleton-extras.cmake
          LIBRARIES ${ADDITIONAL_LIBRARIES})
