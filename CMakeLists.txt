cmake_minimum_required(VERSION 2.8.3)
project(triangulation)
include("../config.cmake")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  cv_bridge
  image_geometry
  image_transport
  genmsg
  nav_msgs
  kalman
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)
find_package(PkgConfig)

catkin_package(
CATKIN_DEPENDS kalman
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS src/triangulation/triangulation_exec.py src/triangulation/triangulation_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
