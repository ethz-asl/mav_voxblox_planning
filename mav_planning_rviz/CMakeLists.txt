cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_rviz)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

# These settings all stolen from visualization_tutorials:
# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC OFF)

set(HEADER_FILES_QT
  include/mav_planning_rviz/pose_widget.h
  include/mav_planning_rviz/planning_panel.h
  include/mav_planning_rviz/edit_button.h
)

set(SRC_FILES
  src/planning_panel.cpp
  src/pose_widget.cpp
  src/edit_button.cpp
  src/planning_interactive_markers.cpp
)

# This plugin includes Qt widgets, so we must include Qt.
# We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES
    ${HEADER_FILES_QT}
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  # make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  QT5_WRAP_CPP(MOC_FILES
    ${HEADER_FILES_QT}
  )
endif()
# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# In case someone else struggles with getting panels to build, see this
# solution:
# https://answers.ros.org/question/215487/could-not-load-panel-in-rviz-pluginlibfactory-the-plugin-for-class/

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})


############
# BINARIES #
############
cs_add_executable(standalone_test
  src/standalone_test.cpp
)
target_link_libraries(standalone_test ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
