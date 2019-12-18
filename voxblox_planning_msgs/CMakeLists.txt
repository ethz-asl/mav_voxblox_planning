cmake_minimum_required(VERSION 2.8.3)
project(voxblox_planning_msgs)

find_package(catkin REQUIRED message_generation geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

#add_message_files(
#FILES
#)
add_service_files(
	FILES
	YawPolicyService.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)