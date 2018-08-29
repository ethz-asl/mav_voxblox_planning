cmake_minimum_required(VERSION 2.8.3)
project(loco)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#  src/trajectory_sampler.cpp
#)

############
# BINARIES #
############
cs_add_executable(loco_main
  src/loco_main.cpp
)

#########
# TESTS #
#########
#catkin_add_gtest(test_cost_functions
#  test/test_cost_functions.cpp
#)
#target_link_libraries(test_cost_functions ${catkin_LIBRARIES} chomp_optimizer)

##########
# EXPORT #
##########
cs_install()
cs_export()
