cmake_minimum_required(VERSION 2.8.3)
project(darknet_ros_3d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  darknet_ros_msgs
  gb_visual_detection_3d_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  roslint
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/darknet_ros_3d/Darknet3D.cpp
  src/darknet_ros_3d/Darknet3DListener.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(darknet3d_node src/darknet3d_node.cpp)
target_link_libraries(darknet3d_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(darknet3d_listener_node test/darknet3d_listener_node.cpp)
target_link_libraries(darknet3d_listener_node ${catkin_LIBRARIES} ${PROJECT_NAME})


roslint_cpp(
  src/darknet_ros_3d/Darknet3D.cpp include/darknet_ros_3d/Darknet3D.h
  src/darknet_ros_3d/Darknet3DListener.cpp include/darknet_ros_3d/Darknet3DListener.h
  src/darknet3d_node.cpp
)

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
