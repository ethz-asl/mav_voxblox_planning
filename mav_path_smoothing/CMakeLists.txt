cmake_minimum_required(VERSION 2.8.3)
project(mav_path_smoothing)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/loco_smoother.cpp
  src/path_smoother_base.cpp
  src/polynomial_smoother.cpp
  src/velocity_ramp_smoother.cpp
)
target_link_libraries(${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
