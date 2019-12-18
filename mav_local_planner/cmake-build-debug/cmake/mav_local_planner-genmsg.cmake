# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mav_local_planner: 0 messages, 1 services")

set(MSG_I_FLAGS "-Imav_msgs:/home/rsteiner/catkin_ws/src/mav_comm/mav_msgs/msg;-Imav_planning_msgs:/home/rsteiner/catkin_ws/src/mav_comm/mav_planning_msgs/msg;-Iroscpp:/opt/ros/melodic/share/roscpp/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itf:/opt/ros/melodic/share/tf/cmake/../msg;-Ivisualization_msgs:/home/rsteiner/catkin_ws/src/common_msgs/visualization_msgs/msg;-Igeometry_msgs:/home/rsteiner/catkin_ws/src/common_msgs/geometry_msgs/msg;-Isensor_msgs:/home/rsteiner/catkin_ws/src/common_msgs/sensor_msgs/msg;-Itrajectory_msgs:/home/rsteiner/catkin_ws/src/common_msgs/trajectory_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mav_local_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_custom_target(_mav_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mav_local_planner" "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(mav_local_planner
  "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_local_planner
)

### Generating Module File
_generate_module_cpp(mav_local_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_local_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mav_local_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mav_local_planner_generate_messages mav_local_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_dependencies(mav_local_planner_generate_messages_cpp _mav_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_local_planner_gencpp)
add_dependencies(mav_local_planner_gencpp mav_local_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_local_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(mav_local_planner
  "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_local_planner
)

### Generating Module File
_generate_module_eus(mav_local_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_local_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mav_local_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mav_local_planner_generate_messages mav_local_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_dependencies(mav_local_planner_generate_messages_eus _mav_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_local_planner_geneus)
add_dependencies(mav_local_planner_geneus mav_local_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_local_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(mav_local_planner
  "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_local_planner
)

### Generating Module File
_generate_module_lisp(mav_local_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_local_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mav_local_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mav_local_planner_generate_messages mav_local_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_dependencies(mav_local_planner_generate_messages_lisp _mav_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_local_planner_genlisp)
add_dependencies(mav_local_planner_genlisp mav_local_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_local_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(mav_local_planner
  "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_local_planner
)

### Generating Module File
_generate_module_nodejs(mav_local_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_local_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mav_local_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mav_local_planner_generate_messages mav_local_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_dependencies(mav_local_planner_generate_messages_nodejs _mav_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_local_planner_gennodejs)
add_dependencies(mav_local_planner_gennodejs mav_local_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_local_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(mav_local_planner
  "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_local_planner
)

### Generating Module File
_generate_module_py(mav_local_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_local_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mav_local_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mav_local_planner_generate_messages mav_local_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rsteiner/catkin_ws/src/mav_voxblox_planning/mav_local_planner/srv/YawPolicyService.srv" NAME_WE)
add_dependencies(mav_local_planner_generate_messages_py _mav_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mav_local_planner_genpy)
add_dependencies(mav_local_planner_genpy mav_local_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mav_local_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mav_local_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp mav_msgs_generate_messages_cpp)
endif()
if(TARGET mav_planning_msgs_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp mav_planning_msgs_generate_messages_cpp)
endif()
if(TARGET roscpp_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp roscpp_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp tf_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(mav_local_planner_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mav_local_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus mav_msgs_generate_messages_eus)
endif()
if(TARGET mav_planning_msgs_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus mav_planning_msgs_generate_messages_eus)
endif()
if(TARGET roscpp_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus roscpp_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus tf_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(mav_local_planner_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mav_local_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp mav_msgs_generate_messages_lisp)
endif()
if(TARGET mav_planning_msgs_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp mav_planning_msgs_generate_messages_lisp)
endif()
if(TARGET roscpp_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp roscpp_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp tf_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(mav_local_planner_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mav_local_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs mav_msgs_generate_messages_nodejs)
endif()
if(TARGET mav_planning_msgs_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs mav_planning_msgs_generate_messages_nodejs)
endif()
if(TARGET roscpp_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs roscpp_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs tf_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(mav_local_planner_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_local_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_local_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mav_local_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET mav_msgs_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py mav_msgs_generate_messages_py)
endif()
if(TARGET mav_planning_msgs_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py mav_planning_msgs_generate_messages_py)
endif()
if(TARGET roscpp_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py roscpp_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py tf_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(mav_local_planner_generate_messages_py visualization_msgs_generate_messages_py)
endif()
