cmake_minimum_required(VERSION 2.8.3)
project(darknet_ros_3d)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  darknet_ros_msgs
  darknet_ros_3d_msgs
  sensor_msgs
  bica
  tf
  pcl_ros
  pcl_conversions
  roslint
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_executable(darknet3d_node
  src/darknet_ros_3d/Darknet3D.cpp
  src/darknet3d_node.cpp)

target_link_libraries(darknet3d_node ${catkin_LIBRARIES})

add_executable(darknet3d_bica_node
  src/darknet_ros_3d/Darknet3D.cpp
  src/darknet3d_bica_node.cpp)

target_link_libraries(darknet3d_bica_node ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")

roslint_cpp(
  src/darknet_ros_3d/Darknet3D.cpp include/darknet_ros_3d/Darknet3D.h
  src/darknet3d_node.cpp
  src/darknet3d_bica_node.cpp
)
