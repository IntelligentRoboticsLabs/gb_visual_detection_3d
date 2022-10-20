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

#include "darknet_ros_3d/Darknet3D.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <memory>
#include <limits>
#include <iostream>
#include <fstream>
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

using std::placeholders::_1;
using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace darknet_ros_3d
{
    Darknet3D::Darknet3D()
        : LifecycleNode("darknet3d_node"), clock_(RCL_SYSTEM_TIME),
          tfBuffer_(std::make_shared<rclcpp::Clock>(clock_)), tfListener_(tfBuffer_, true),
          pc_received_(false)
    {
        // Init Params

        this->declare_parameter("darknet_ros_topic", "/darknet_ros/bounding_boxes");
        this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");
        this->declare_parameter("point_cloud_topic", "/velodyne_points");
        this->declare_parameter("working_frame", "camera_link");
        this->declare_parameter("maximum_detection_threshold", 0.3f);
        this->declare_parameter("minimum_probability", 0.3f);
        // this->declare_parameter("interested_classes", "/darknet_ros/check_for_objects");

        this->configure();

        pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 1, std::bind(&Darknet3D::pointCloudCb, this, std::placeholders::_1));

        darknet_ros_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            input_bbx_topic_, 1, std::bind(&Darknet3D::darknetCb, this, std::placeholders::_1));

        darknet3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
            output_bbx3d_topic_, 100);

        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/darknet_ros_3d/markers", 1);

        view_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/darknet_ros_3d/view", 1);

        view_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/darknet_ros_3d/view_points", 1);

        human_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/darknet_ros_3d/human_points", 1);

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

    pcl::PointCloud<pcl::PointXYZ>
    Darknet3D::calculate_view_points(
        pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        // In 3D space use this constant to calculate the field of view lines for 
        // x = constant * y and x = -constant * y
        // This line constant is calibrated for the camera
        double line_constant = 1.78571;
        // First we check which points are in our field of view
        std::vector<pcl::PointXYZ> points_in_view;
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(cloud, *new_cloud);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (std::size_t i = 0; i < new_cloud->size(); i++) {
            int xx = new_cloud->points[i].x;
            int yy = new_cloud->points[i].y;
            if (xx >= 0 && (line_constant * std::abs(yy)) < xx && xx < 20) {
                // We know the point is in our field of view
                if (std::isnan(xx))
                {
                    continue;
                }

                points_in_view.push_back(new_cloud->points[i]);              
            } else {
                inliers->indices.push_back(i);
            }
        }

        // filter the cloud to only include the points in the camera view
        extract.setInputCloud(new_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*new_cloud);
        // RCLCPP_INFO(this->get_logger(), "Cloud size After: %s", std::to_string(cloud.size()).c_str());

        // convert all the points in view to a raw sensor messages point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*new_cloud, pcl_pc2);
        sensor_msgs::msg::PointCloud2 raw_pointcloud;
        pcl_conversions::fromPCL(pcl_pc2, raw_pointcloud);

        // publish the point cloud to the ros topic
        if (view_points_pub_->is_activated())
        {
            view_points_pub_->publish(raw_pointcloud);
        }

        // return the vector containing the points in the view
        return *new_cloud;
    }

    void
    Darknet3D::calculate_boxes(
        sensor_msgs::msg::PointCloud2 cloud_pc2,
        pcl::PointCloud<pcl::PointXYZ> cloud,
        gb_visual_detection_3d_msgs::msg::BoundingBoxes3d *boxes)
    {
        boxes->header.stamp = cloud_pc2.header.stamp;
        boxes->header.frame_id = cloud_pc2.header.frame_id;

        pcl::PointCloud<pcl::PointXYZ> points_in_view = calculate_view_points(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(points_in_view, *new_cloud);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        double line_constant_h = 1.78571;
        double line_constant_v = 3.1746;
        double camera_width = 1920;
        double camera_height = 1080;
        // Loop through the bounding boxes
        for (auto bbx : original_bboxes_)
        {
            if (bbx.probability < minimum_probability_ || bbx.class_id == "dont_show")
            {
                // RCLCPP_INFO(this->get_logger(), "Skipped bounding box");
                continue;
            }

            int center_x, center_y;
            center_x = (bbx.xmax + bbx.xmin) / 2;
            center_y = (bbx.ymax + bbx.ymin) / 2;

            float maxx, minx, maxy, miny, maxz, minz;
            maxx = maxy = maxz = -std::numeric_limits<float>::max();
            minx = miny = minz = std::numeric_limits<float>::max();

            // RCLCPP_INFO(this->get_logger(), "Start point loop");

            for (std::size_t i = 0; i < new_cloud->size(); i++) {
                float xx = new_cloud->points[i].x;
                float yy = new_cloud->points[i].y;
                float zz = new_cloud->points[i].z;
                
                if (bbx.xmin == 0 || bbx.xmax == 0) {
                    RCLCPP_INFO(this->get_logger(), "xmin or xmax had value of 0, skipping");
                    continue;
                }

                // the width of the horizontal camera view in XYZ space
                double lidar_width = (xx/line_constant_h)*2;
                // the scaling factor to convert XYZ to pixels
                double scale = camera_width/lidar_width;
                // distance in meters lidar point is from left edge of view
                double point_pixel_edge_h = (xx/line_constant_h) - yy;
                // The horizontal pixel approximation of the lidar point
                double lidar_pixel_loc_h = point_pixel_edge_h * scale;

                double lidar_height = (xx/line_constant_v)*2;
                double scale2 = camera_height/lidar_height;
                // distance from bottom view of camera
                double point_pixel_edge_v = lidar_height - ((xx/line_constant_v) - zz);
                double lidar_pixel_loc_v = point_pixel_edge_v * scale2;
                // RCLCPP_INFO(this->get_logger(), "Lidar loc %f Edge %f Scale %f Xmax %ld Xmin %ld",
                //     lidar_pixel_loc, point_pixel_edge, scale, bbx.xmax, bbx.xmin);
                // if (lidar_pixel_loc_h >= bbx.xmin && lidar_pixel_loc_h <= bbx.xmax && 
                //     lidar_pixel_loc_v >= bbx.ymin && lidar_pixel_loc_v <= bbx.ymax) {
                if (lidar_pixel_loc_h >= bbx.xmin && lidar_pixel_loc_h <= bbx.xmax && zz > -1.5) {
                    // We know the point is on the human
                    //RCLCPP_INFO(this->get_logger(), "Detected point on person");

                    maxx = std::max(xx, maxx);
                    maxy = std::max(yy, maxy);
                    maxz = std::max(zz, maxz);
                    minx = std::min(xx, minx);
                    miny = std::min(yy, miny);
                    minz = std::min(zz, minz);
                } else {
                    inliers->indices.push_back(i);
                }
            }

            // filter the cloud to only include the points in the camera view
            extract.setInputCloud(new_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*new_cloud);

            // convert all the points in view to a raw sensor messages point cloud
            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*new_cloud, pcl_pc2);
            sensor_msgs::msg::PointCloud2 raw_pointcloud;
            pcl_conversions::fromPCL(pcl_pc2, raw_pointcloud);

            // publish the point cloud to the ros topic
            if (human_points_pub_->is_activated())
            {
                human_points_pub_->publish(raw_pointcloud);
            }

            //RCLCPP_INFO(this->get_logger(), "Finished point loop");

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

            // RCLCPP_INFO(this->get_logger(), "Calculate bbox finished ended");
            
        }
    }

    void
    Darknet3D::publish_markers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes)
    {
        visualization_msgs::msg::MarkerArray msg;
        visualization_msgs::msg::MarkerArray view;

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
            bbx_marker.lifetime = rclcpp::Duration::from_seconds(1.0); //
            bbx_marker.text = bb.object_name;

            msg.markers.push_back(bbx_marker);
        }

        for (int i = 0; i < 4; i++) {
            visualization_msgs::msg::Marker line;
            geometry_msgs::msg::Point point1;
            point1.x = 0;
            point1.y = 0;
            point1.z = 0;
            geometry_msgs::msg::Point point2;
            point2.x = 50;
            if (i == 0) {
                point2.y = 28;
                point2.z = 0;
            } else if (i == 1) {
                point2.y = -28;
                point2.z = 0;
            } else if (i == 2) {
                point2.y = 0;
                point2.z = 15.75;
            } else {
                point2.y = 0;
                point2.z = -15.75;
            }
            
            std::vector<geometry_msgs::msg::Point> point_list{point1, point2}; 

            line.header.frame_id = working_frame_;
            line.header.stamp = boxes.header.stamp;
            line.ns = "darknet3d";
            line.id = i;
            line.type = visualization_msgs::msg::Marker::ARROW;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.frame_locked = false;
            line.points = point_list;
            line.color.b = 229;
            line.color.g = 204;
            line.color.r = 255.0;
            line.color.a = 0.4;
            line.scale.x = 0.3;
            line.scale.y = 0.5;
            line.scale.z = 0;
            line.lifetime = rclcpp::Duration::from_seconds(1.0); //
            if (i == 0) {
                line.text = "Left";
            } else {
                line.text = "Right";
            }

            view.markers.push_back(line);
        }

        if (markers_pub_->is_activated())
        {
            markers_pub_->publish(msg);
        }
        if (view_pub_->is_activated())
        {
            view_pub_->publish(view);
        }
    }

    void
    Darknet3D::update()
    {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            return;
        }

        if ((clock_.now() - last_detection_ts_).seconds() > 2.0 || !pc_received_)
        {
            return;
        }

        sensor_msgs::msg::PointCloud2 local_pointcloud;
        geometry_msgs::msg::TransformStamped camera_transform;
        geometry_msgs::msg::TransformStamped base_transform;
        gb_visual_detection_3d_msgs::msg::BoundingBoxes3d msg;

        try
        {
            // remove the '/' from the point cloud frame id
            std::string frame_id = point_cloud_.header.frame_id.substr(1, -1);
            camera_transform = tfBuffer_.lookupTransform(working_frame_, frame_id,
                                                  point_cloud_.header.stamp, tf2::durationFromSec(2.0));
            base_transform = tfBuffer_.lookupTransform("base_link", working_frame_, 
                                                point_cloud_.header.stamp, tf2::durationFromSec(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
                         ex.what(), "quitting callback");
            return;
        }
        tf2::doTransform<sensor_msgs::msg::PointCloud2>(point_cloud_, local_pointcloud, camera_transform);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(local_pointcloud, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // int point_cloud_size = cloud->size();
        // if (point_cloud_size < 100000000) {
        //     RCLCPP_INFO(this->get_logger(), "Skipping Point cloud size of %s", std::to_string(point_cloud_size).c_str());
        //     RCLCPP_INFO(this->get_logger(), "Height: %s", std::to_string(cloud->height).c_str());
        //     RCLCPP_INFO(this->get_logger(), "Width: %s", std::to_string(cloud->width).c_str());
        //     return;
        // }
        
        // tf2::Stamped<tf2::Transform> stamped_transform;
        // tf2::fromMsg(base_transform, stamped_transform);
        // tf2::Vector3 vector = stamped_transform.getOrigin();
        // double x = vector.getX();
        // double y = vector.getY();
        // double z = vector.getZ();
        // RCLCPP_INFO(this->get_logger(), "Transform base X %d Y %d Z %d", x, y, z);
        // return;
        calculate_boxes(local_pointcloud, *cloud, &msg);
        publish_markers(msg);

        if (darknet3d_pub_->is_activated())
        {
            darknet3d_pub_->publish(msg);
        }
    }

    CallbackReturnT
    Darknet3D::on_configure(const rclcpp_lifecycle::State &state)
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
    Darknet3D::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
                    this->get_name(), state.label().c_str());

        darknet3d_pub_->on_activate();
        markers_pub_->on_activate();
        view_pub_->on_activate();
        view_points_pub_->on_activate();
        human_points_pub_->on_activate();

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    Darknet3D::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
                    this->get_name(), state.label().c_str());

        darknet3d_pub_->on_deactivate();
        markers_pub_->on_deactivate();
        view_pub_->on_deactivate();
        view_points_pub_->on_deactivate();
        human_points_pub_->on_deactivate();

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    Darknet3D::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
                    this->get_name(), state.label().c_str());

        darknet3d_pub_.reset();
        markers_pub_.reset();
        view_pub_.reset();
        view_points_pub_.reset();
        human_points_pub_.reset();

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    Darknet3D::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
                    this->get_name(), state.label().c_str());

        darknet3d_pub_.reset();
        markers_pub_.reset();
        view_pub_.reset();
        view_points_pub_.reset();
        human_points_pub_.reset();

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT
    Darknet3D::on_error(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
                    this->get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

} // namespace darknet_ros_3d