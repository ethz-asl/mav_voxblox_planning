# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "loco_planner;mav_msgs;mav_planning_common;mav_trajectory_generation_ros;roscpp;std_msgs;tf;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmav_path_smoothing".split(';') if "-lmav_path_smoothing" != "" else []
PROJECT_NAME = "mav_path_smoothing"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
