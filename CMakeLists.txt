cmake_minimum_required(VERSION 2.8.3)
project(triangulation)
include("../config.cmake")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
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
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

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

# Triangulation
add_library(triangulation_node src/triangulation/triangulation_node.cpp)
target_link_libraries(triangulation_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(triangulation src/triangulation/triangulation.cpp)
target_link_libraries(triangulation
  triangulation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
