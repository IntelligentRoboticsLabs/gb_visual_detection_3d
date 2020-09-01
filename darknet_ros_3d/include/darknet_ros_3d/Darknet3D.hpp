// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es */

#ifndef DARKNET_ROS_3D__DARKNET3D_HPP_
#define DARKNET_ROS_3D__DARKNET3D_HPP_

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <vector>
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"

namespace darknet_ros_3d
{

class Darknet3D : public rclcpp_lifecycle::LifecycleNode
{
public:
  Darknet3D();
  void update();

private:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void darknetCb(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg);
  void calculate_boxes(
    sensor_msgs::msg::PointCloud2 cloud_pc2, sensor_msgs::msg::PointCloud cloud_pc,
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes);

  void publish_markers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_sub_;
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr darknet_ros_sub_;

  rclcpp_lifecycle::LifecyclePublisher
  <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr darknet3d_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  rclcpp::Clock clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  sensor_msgs::msg::PointCloud2 point_cloud_;
  rclcpp::Time last_detection_ts_;
  std::string input_bbx_topic_;
  std::string output_bbx3d_topic_;
  std::string pointcloud_topic_;
  std::string working_frame_;
  std::vector<std::string> interested_classes_ = {};
  std::vector<darknet_ros_msgs::msg::BoundingBox> original_bboxes_;
  float maximum_detection_threshold_, minimum_probability_;
  bool pc_received_;
};

}  // namespace darknet_ros_3d

#endif  // DARKNET_ROS_3D__DARKNET3D_HPP_
