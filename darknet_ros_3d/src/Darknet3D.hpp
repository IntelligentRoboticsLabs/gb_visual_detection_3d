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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_geometry/pinhole_camera_model.h>
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
  void infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void cameraCb(const sensor_msgs::msg::Image::SharedPtr msg);
  void calculate_boxes(
    sensor_msgs::msg::PointCloud2 cloud_pc2, pcl::PointCloud<pcl::PointXYZ> cloud,
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes,
    float ground_z);
  void publish_markers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes);
  pcl::PointCloud<pcl::PointXYZRGB>
    augment_pointcloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud,
    gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg,
    image_geometry::PinholeCameraModel cam_model);
  
  pcl::PointCloud<pcl::PointXYZ> calculate_view_points(
        pcl::PointCloud<pcl::PointXYZ> cloud,
        image_geometry::PinholeCameraModel cam_model);



  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_sub_;
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr darknet_ros_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp_lifecycle::LifecyclePublisher
  <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr darknet3d_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <sensor_msgs::msg::PointCloud2>::SharedPtr view_points_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <sensor_msgs::msg::PointCloud2>::SharedPtr human_points_pub_;


  rclcpp::Clock clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  sensor_msgs::msg::PointCloud2 point_cloud_;
  sensor_msgs::msg::CameraInfo camera_info_;
  sensor_msgs::msg::Image camera_image_;
  rclcpp::Time last_detection_ts_;
  std::string input_bbx_topic_;
  std::string output_bbx3d_topic_;
  std::string output_markers_topic_;
  std::string output_view_points_topic_;
  std::string output_human_points_topic_;
  std::string pointcloud_topic_;
  std::string camera_info_topic_;
  std::string camera_image_topic_;
  std::string working_frame_;
  std::string transform_frame_;
  std::vector<std::string> interested_classes_ = {};
  std::vector<darknet_ros_msgs::msg::BoundingBox> original_bboxes_;
  float ground_detection_threshold_, minimum_probability_, bbx3d_tolerance_;
  bool pc_received_;
};

}  // namespace darknet_ros_3d

#endif  // DARKNET_ROS_3D__DARKNET3D_HPP_
