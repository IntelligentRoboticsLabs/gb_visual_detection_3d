/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#ifndef DARKNET_ROS_3D__DARKNET3D_H_
#define DARKNET_ROS_3D__DARKNET3D_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

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

  void init_params();
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

}

#endif // DARKNET_ROS_3D__DARKNET3D_H_
