cmake_minimum_required(VERSION 2.8.3)
project(mav_path_smoothing)

find_package(catkin_simple REQUIRED)
find_package(OMPL REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/path_smoother_base.cpp
  src/velocity_ramp_smoother.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
