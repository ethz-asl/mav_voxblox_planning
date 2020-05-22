# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "voxblox;voxblox_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lvoxblox_skeleton_proto;-lvoxblox_skeleton;/usr/lib/x86_64-linux-gnu/libprotobuf-lite.so;/usr/lib/x86_64-linux-gnu/libprotoc.so;/usr/lib/x86_64-linux-gnu/libprotobuf.so".split(';') if "-lvoxblox_skeleton_proto;-lvoxblox_skeleton;/usr/lib/x86_64-linux-gnu/libprotobuf-lite.so;/usr/lib/x86_64-linux-gnu/libprotoc.so;/usr/lib/x86_64-linux-gnu/libprotobuf.so" != "" else []
PROJECT_NAME = "voxblox_skeleton"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
