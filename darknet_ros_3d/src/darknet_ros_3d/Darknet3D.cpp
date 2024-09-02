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
#include <chrono>
namespace darknet_ros_3d
{

Darknet3D::Darknet3D():
   nh_("~")
{
  initParams();

  darknet3d_pub_ = nh_.advertise<gb_visual_detection_3d_msgs::BoundingBoxes3d>(output_bbx3d_topic_, 100);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/darknet_ros_3d/markers", 100);

  yolo_sub_ = nh_.subscribe(input_bbx_topic_, 1, &Darknet3D::darknetCb, this);
  pointCloud_sub_.subscribe(nh_, pointcloud_topic_, 10);
  pointcloud_cache_.setCacheSize(100);
  pointcloud_cache_.connectInput(pointCloud_sub_);
  
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
Darknet3D::darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  // Retrieve the closest PointCloud2 message to the image's timestamp not the detection
  ros::Time image_stamp = msg->image_header.stamp;
  sensor_msgs::PointCloud2ConstPtr closest_pointcloud_msg = pointcloud_cache_.getElemBeforeTime(image_stamp);

  if (closest_pointcloud_msg)
    {
        // ROS_INFO("Found a matching PointCloud2 message with timestamp: %f", closest_pointcloud_msg->header.stamp.toSec());
        // Process the point cloud data
        if ((darknet3d_pub_.getNumSubscribers() == 0) &&
            (markers_pub_.getNumSubscribers() == 0))
            return;

        sensor_msgs::PointCloud2 local_pointcloud;

        try
        {
           pcl_ros::transformPointCloud(working_frame_, *closest_pointcloud_msg, local_pointcloud, tfListener_);
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
        }

        

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(local_pointcloud, *pcrgb);

        gb_visual_detection_3d_msgs::BoundingBoxes3d boxes3d_msg;
        

        calculate_boxes(local_pointcloud, pcrgb, msg, &boxes3d_msg);

        darknet3d_pub_.publish(boxes3d_msg);

        publish_markers(boxes3d_msg);
    }

    else
    {
        ROS_WARN("No matching PointCloud2 message found for image timestamp: %f", image_stamp.toSec());
    }

}

void
Darknet3D::calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
    const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes2d,
    gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes)
{
  boxes->header.stamp = cloud_pc2.header.stamp;
  boxes->header.frame_id = working_frame_;

  for (const auto&  bbx : boxes2d->bounding_boxes)
  {
    if ((bbx.probability < minimum_probability_) ||
        (std::find(interested_classes_.begin(), interested_classes_.end(), bbx.Class) == interested_classes_.end()))
    {
      continue;
    }

    pcl::PointXYZRGB center_point = compute_center_point(cloud_pc2, cloud_pcl, bbx);

    float maxx, minx, maxy, miny, maxz, minz;

    maxx = maxy = maxz =  -std::numeric_limits<float>::max();
    minx = miny = minz =  std::numeric_limits<float>::max();
    
    int pcl_index = 0;
    for (int i = bbx.xmin; i < bbx.xmax; i++)
      for (int j = bbx.ymin; j < bbx.ymax; j++)
      {
        pcl_index = (j* cloud_pc2.width) + i;
        pcl::PointXYZRGB point =  cloud_pcl->at(pcl_index);

        if (std::isnan(point.x)){
          continue;
        }
          

        if (fabs(point.x - center_point.x) > mininum_detection_thereshold_){
          continue;
        }
          

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

pcl::PointXYZRGB Darknet3D::compute_center_point(const sensor_msgs::PointCloud2& cloud_pc2,
                                                 const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
                                                 const darknet_ros_msgs::BoundingBox& box2d) 
{
    auto start = std::chrono::high_resolution_clock::now();
    int center_x = (box2d.xmax + box2d.xmin) / 2;
    int center_y = (box2d.ymax + box2d.ymin) / 2;

    int width = box2d.xmax - box2d.xmin;
    int height = box2d.ymax - box2d.ymin;

    // Convert num_samples to be in terms of height and width percentages
    int num_samples = 500; // Example number of samples
    float width_percentage = 0.55f;  // 55% of the bounding box width
    float height_percentage = 0.35f; // 35% of the bounding box height

    int width_step = static_cast<int>(width * width_percentage) / num_samples;
    int height_step = static_cast<int>(height * height_percentage) / num_samples;

    std::vector<pcl::PointXYZRGB> points;

    // Sampling in the region defined by the bounding box
    for (int i = -num_samples / 2; i <= num_samples / 2; ++i) {
        for (int j = -num_samples / 2; j <= num_samples / 2; ++j) {
            int new_x = center_x + i * width_step;
            int new_y = center_y + j * height_step;

            // Ensure the new points are within the image bounds
            if (new_x < 0 || new_x >= cloud_pc2.width || new_y < 0 || new_y >= cloud_pc2.height) {
                continue;
            }

            int pcl_index = (new_y * cloud_pc2.width) + new_x;
            pcl::PointXYZRGB point = cloud_pcl->at(pcl_index);

            // Check if the point is valid
            if (!std::isnan(point.x)) {
                points.push_back(point);
            }
        }
    }

    // If no valid points were found, return a default invalid point
    if (points.empty()) {
        return pcl::PointXYZRGB();
    }

    // Find the point with the minimum x value
    pcl::PointXYZRGB min_x_point = points[0];
    for (const auto& pt : points) {
        if (pt.x < min_x_point.x) {
            min_x_point = pt;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "All the processing took " << duration.count()
              << " milliseconds." << std::endl;
    return min_x_point;
}


};  // namespace darknet_ros_3d
