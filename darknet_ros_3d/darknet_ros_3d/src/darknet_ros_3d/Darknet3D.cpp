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

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "darknet_ros_3d/Darknet3D.h"
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"


using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace darknet_ros_3d
{
  Darknet3D::Darknet3D():
    LifecycleNode("darknet3d_node"), clock_(RCL_SYSTEM_TIME),
    tfBuffer_(std::make_shared<rclcpp::Clock>(clock_)), tfListener_(tfBuffer_, true),
    pc_received_(false)
  {
    // init params
    this->declare_parameter("darknet_ros_topic", "/darknet_ros/bounding_boxes");
    this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");
    this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");
    this->declare_parameter("working_frame", "camera_link");
    this->declare_parameter("maximum_detection_threshold", 0.3f);
    this->declare_parameter("minimum_probability", 0.3f);
    this->declare_parameter("interested_classes");

    this->configure();

    pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 1,
            std::bind(&Darknet3D::pointCloudCb, this, std::placeholders::_1));

    darknet_ros_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>
                      (input_bbx_topic_, 1, std::bind(&Darknet3D::darknetCb,
                      this, std::placeholders::_1));

    darknet3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>
                      (output_bbx3d_topic_, 100);

    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>
                    ("/darknet_ros_3d/markers", 1);

    last_detection_ts_ = clock_.now();

    this->activate();
  }

  void
  Darknet3D::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    point_cloud_ = *msg;
    pc_received_ = true;
  }

  void
  Darknet3D::darknetCb(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  {
    original_bboxes_ = msg->bounding_boxes;
    last_detection_ts_ = clock_.now();
  }

  void
  Darknet3D::calculate_boxes(sensor_msgs::msg::PointCloud2 cloud_pc2,
    sensor_msgs::msg::PointCloud cloud_pc,
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d *boxes)
  {
    boxes->header.stamp = cloud_pc2.header.stamp;
    boxes->header.frame_id = cloud_pc2.header.frame_id;

    for (auto bbx : original_bboxes_)
    {
      if ((bbx.probability < minimum_probability_) ||
          (std::find(interested_classes_.begin(), interested_classes_.end(),
            bbx.class_id) == interested_classes_.end()))
      {
        continue;
      }

      int center_x, center_y;

      center_x = (bbx.xmax + bbx.xmin) / 2;
      center_y = (bbx.ymax + bbx.ymin) / 2;

      int pc_index = (center_y* cloud_pc2.width) + center_x;
      geometry_msgs::msg::Point32 center_point =  cloud_pc.points[pc_index];

      if (std::isnan(center_point.x))
        continue;

      float maxx, minx, maxy, miny, maxz, minz;

      maxx = maxy = maxz =  -std::numeric_limits<float>::max();
      minx = miny = minz =  std::numeric_limits<float>::max();

      for (int i = bbx.xmin; i < bbx.xmax; i++)
      {
        for (int j = bbx.ymin; j < bbx.ymax; j++)
        {
          pc_index = (j* cloud_pc2.width) + i;
          geometry_msgs::msg::Point32 point =  cloud_pc.points[pc_index];

          if (std::isnan(point.x))
            continue;

          if (fabs(point.x - center_point.x) > maximum_detection_threshold_)
            continue;

          maxx = std::max(point.x, maxx);
          maxy = std::max(point.y, maxy);
          maxz = std::max(point.z, maxz);
          minx = std::min(point.x, minx);
          miny = std::min(point.y, miny);
          minz = std::min(point.z, minz);
        }
      }

      gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg;
      bbx_msg.object_name = bbx.class_id;
      bbx_msg.probability = bbx.probability;

      bbx_msg.xmin = minx;
      bbx_msg.xmax = maxx;
      bbx_msg.ymin = miny;
      bbx_msg.ymax = maxy;
      bbx_msg.zmin = minz;
      bbx_msg.zmax = maxz;

      boxes->bounding_boxes.push_back(bbx_msg);
    }
  }

  void
  Darknet3D::publish_markers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes)
  {
    visualization_msgs::msg::MarkerArray msg;

    int counter_id = 0;
    for (auto bb : boxes.bounding_boxes)
    {
      visualization_msgs::msg::Marker bbx_marker;

      bbx_marker.header.frame_id = working_frame_;
      bbx_marker.header.stamp = boxes.header.stamp;
      bbx_marker.ns = "darknet3d";
      bbx_marker.id = counter_id++;
      bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
      bbx_marker.action = visualization_msgs::msg::Marker::ADD;
      bbx_marker.frame_locked = false;
      bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
      bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
      bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;
      bbx_marker.pose.orientation.x = 0.0;
      bbx_marker.pose.orientation.y = 0.0;
      bbx_marker.pose.orientation.z = 0.0;
      bbx_marker.pose.orientation.w = 1.0;
      bbx_marker.scale.x = (bb.xmax - bb.xmin);
      bbx_marker.scale.y = (bb.ymax - bb.ymin);
      bbx_marker.scale.z = (bb.zmax - bb.zmin);
      bbx_marker.color.b = 0;
      bbx_marker.color.g = bb.probability * 255.0;
      bbx_marker.color.r = (1.0 - bb.probability) * 255.0;
      bbx_marker.color.a = 0.4;
      bbx_marker.lifetime = rclcpp::Duration(1.0);
      bbx_marker.text = bb.object_name;

      msg.markers.push_back(bbx_marker);
    }

    if(markers_pub_->is_activated())
      markers_pub_->publish(msg);
  }

  void
  Darknet3D::update()
  {
    if(this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      return;

    if((clock_.now() - last_detection_ts_).seconds() > 2.0 || ! pc_received_)
      return;

    sensor_msgs::msg::PointCloud2 local_pointcloud;
    geometry_msgs::msg::TransformStamped transform;
    sensor_msgs::msg::PointCloud cloud_pc;
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d msg;

    try
    {
      transform = tfBuffer_.lookupTransform(working_frame_, point_cloud_.header.frame_id,
          point_cloud_.header.stamp, tf2::durationFromSec(2.0));

    }
    catch(tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
                    ex.what(), "quitting callback");
      return;
    }
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(point_cloud_, local_pointcloud, transform);
    sensor_msgs::convertPointCloud2ToPointCloud(local_pointcloud, cloud_pc);

    calculate_boxes(local_pointcloud, cloud_pc, &msg);
    publish_markers(msg);

    if(darknet3d_pub_->is_activated())
      darknet3d_pub_->publish(msg);

  }

  CallbackReturnT
  Darknet3D::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
                this->get_name(), state.label().c_str());

    this->get_parameter("darknet_ros_topic", input_bbx_topic_);
    this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
    this->get_parameter("point_cloud_topic", pointcloud_topic_);
    this->get_parameter("working_frame", working_frame_);
    this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
    this->get_parameter("minimum_probability", minimum_probability_);
    this->get_parameter("interested_classes", interested_classes_);

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  Darknet3D::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
                this->get_name(), state.label().c_str());

    darknet3d_pub_->on_activate();
    markers_pub_->on_activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  Darknet3D::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
                this->get_name(), state.label().c_str());

    darknet3d_pub_->on_deactivate();
    markers_pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  Darknet3D::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
                this->get_name(), state.label().c_str());

    darknet3d_pub_.reset();
    markers_pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  Darknet3D::on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
                this->get_name(), state.label().c_str());

    darknet3d_pub_.reset();
    markers_pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  Darknet3D::on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
                this->get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }
} //end namespace darknet_ros_3ds
