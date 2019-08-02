cmake_minimum_required(VERSION 2.8.3)
project(darknet_ros_3d_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
 FILES
 BoundingBoxes3d.msg
 BoundingBox3d.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
