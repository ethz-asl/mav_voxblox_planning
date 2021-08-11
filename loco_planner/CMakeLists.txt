cmake_minimum_required(VERSION 2.8.3)
project(loco_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

############
# BINARIES #
############
cs_add_executable(loco_main
  src/loco_main.cpp
)

#########
# TESTS #
#########
catkin_add_gtest(test_loco
  test/test_loco.cpp
)
target_link_libraries(test_loco ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
