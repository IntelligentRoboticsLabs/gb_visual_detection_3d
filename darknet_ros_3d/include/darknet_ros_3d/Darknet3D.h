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

#ifndef DARKNET_ROS_3D_DARKNET3D_H
#define DARKNET_ROS_3D_DARKNET3D_H

#include <ros/ros.h>

#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <vector>
#include <string>

namespace darknet_ros_3d
{

class Darknet3D
{
public:
  Darknet3D();

  virtual void update();

private:
  void initParams();
  void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  void publish_markers(const gb_visual_detection_3d_msgs::BoundingBoxes3d& boxes);

  void calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_pcl,
      gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes);

  ros::NodeHandle nh_;
  ros::Subscriber yolo_sub_, pointCloud_sub_;
  ros::Publisher darknet3d_pub_, markers_pub_;
  tf::TransformListener tfListener_;

  std::vector<darknet_ros_msgs::BoundingBox> original_bboxes_;
  sensor_msgs::PointCloud2 point_cloud_;
  ros::Time last_detection_ts_;

  std::string input_bbx_topic_;
  std::string output_bbx3d_topic_;
  std::string pointcloud_topic_;
  std::string working_frame_;
  std::vector<std::string> interested_classes_;
  float mininum_detection_thereshold_, minimum_probability_;
};

};  // namespace darknet_ros_3d

#endif  // DARKNET_ROS_3D_DARKNET3D_H
