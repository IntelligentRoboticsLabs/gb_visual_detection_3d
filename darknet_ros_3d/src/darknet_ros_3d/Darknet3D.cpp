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

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es  */

#include "darknet_ros_3d/Darknet3D.h"

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <algorithm>

namespace darknet_ros_3d
{

Darknet3D::Darknet3D():
  nh_("~")
{
  initParams();

  darknet3d_pub_ = nh_.advertise<gb_visual_detection_3d_msgs::BoundingBoxes3d>(output_bbx3d_topic_, 100);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/darknet_ros_3d/markers", 100);

  yolo_sub_ = nh_.subscribe(input_bbx_topic_, 1, &Darknet3D::darknetCb, this);
  pointCloud_sub_ = nh_.subscribe(pointcloud_topic_, 1, &Darknet3D::pointCloudCb, this);

  last_detection_ts_ = ros::Time::now() - ros::Duration(60.0);
}

void
Darknet3D::initParams()
{
  input_bbx_topic_ = "/darknet_ros/bounding_boxes";
  output_bbx3d_topic_ = "/darknet_ros_3d/bounding_boxes";
  pointcloud_topic_ = "/camera/depth_registered/points";
  working_frame_ = "/camera_link";
  mininum_detection_thereshold_ = 0.5f;
  minimum_probability_ = 0.3f;

  nh_.param("darknet_ros_topic", input_bbx_topic_, input_bbx_topic_);
  nh_.param("output_bbx3d_topic", output_bbx3d_topic_, output_bbx3d_topic_);
  nh_.param("point_cloud_topic", pointcloud_topic_, pointcloud_topic_);
  nh_.param("working_frame", working_frame_, working_frame_);
  nh_.param("mininum_detection_thereshold", mininum_detection_thereshold_, mininum_detection_thereshold_);
  nh_.param("minimum_probability", minimum_probability_, minimum_probability_);
  nh_.param("interested_classes", interested_classes_, interested_classes_);
}

void
Darknet3D::pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  point_cloud_ = *msg;
}

void
Darknet3D::darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  last_detection_ts_ = ros::Time::now();
  original_bboxes_ = msg->bounding_boxes;
}

void
Darknet3D::calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_pcl,
    gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes)
{
  boxes->header.stamp = cloud_pc2.header.stamp;
  boxes->header.frame_id = working_frame_;

  for (auto bbx : original_bboxes_)
  {
    if ((bbx.probability < minimum_probability_) ||
        (std::find(interested_classes_.begin(), interested_classes_.end(), bbx.Class) == interested_classes_.end()))
    {
      continue;
    }

    int center_x, center_y;

    center_x = (bbx.xmax + bbx.xmin) / 2;
    center_y = (bbx.ymax + bbx.ymin) / 2;

    int pcl_index = (center_y* cloud_pc2.width) + center_x;
    pcl::PointXYZRGB center_point =  cloud_pcl->at(pcl_index);

    if (std::isnan(center_point.x))
      continue;

    float maxx, minx, maxy, miny, maxz, minz;

    maxx = maxy = maxz =  -std::numeric_limits<float>::max();
    minx = miny = minz =  std::numeric_limits<float>::max();

    for (int i = bbx.xmin; i < bbx.xmax; i++)
      for (int j = bbx.ymin; j < bbx.ymax; j++)
      {
        pcl_index = (j* cloud_pc2.width) + i;
        pcl::PointXYZRGB point =  cloud_pcl->at(pcl_index);

        if (std::isnan(point.x))
          continue;

        if (fabs(point.x - center_point.x) > mininum_detection_thereshold_)
          continue;

        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
      }

    gb_visual_detection_3d_msgs::BoundingBox3d bbx_msg;
    bbx_msg.Class = bbx.Class;
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
Darknet3D::update()
{
  if ((ros::Time::now() - last_detection_ts_).toSec() > 2.0)
    return;

  if ((darknet3d_pub_.getNumSubscribers() == 0) &&
      (markers_pub_.getNumSubscribers() == 0))
    return;

  sensor_msgs::PointCloud2 local_pointcloud;

  try
  {
    pcl_ros::transformPointCloud(working_frame_, point_cloud_, local_pointcloud, tfListener_);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(local_pointcloud, *pcrgb);

  gb_visual_detection_3d_msgs::BoundingBoxes3d msg;

  calculate_boxes(local_pointcloud, pcrgb, &msg);

  darknet3d_pub_.publish(msg);

  publish_markers(msg);
}

void
Darknet3D::publish_markers(const gb_visual_detection_3d_msgs::BoundingBoxes3d& boxes)
{
  visualization_msgs::MarkerArray msg;

  int counter_id = 0;
  for (auto bb : boxes.bounding_boxes)
  {
    visualization_msgs::Marker bbx_marker;

    bbx_marker.header.frame_id = boxes.header.frame_id;
    bbx_marker.header.stamp = boxes.header.stamp;
    bbx_marker.ns = "darknet3d";
    bbx_marker.id = counter_id++;
    bbx_marker.type = visualization_msgs::Marker::CUBE;
    bbx_marker.action = visualization_msgs::Marker::ADD;
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
    bbx_marker.lifetime = ros::Duration(0.5);

    msg.markers.push_back(bbx_marker);
  }

  markers_pub_.publish(msg);
}

};  // namespace darknet_ros_3d
