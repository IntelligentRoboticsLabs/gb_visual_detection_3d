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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>
#include <algorithm>
#include <memory>
#include <limits>
#include <iostream>
#include <fstream>
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

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
        this->declare_parameter("output_markers_topic", "/darknet_ros_3d/markers");
        this->declare_parameter("output_view_points_topic", "/darknet_ros_3d/view_points");
        this->declare_parameter("output_human_points_topic", "/darknet_ros_3d/human_points");
        this->declare_parameter("point_cloud_topic", "/velodyne_points");
        this->declare_parameter("camera_info_topic", "/camera/camera_info");
        this->declare_parameter("camera_image_topic", "/darknet_ros/detection_image");
        this->declare_parameter("raw_camera_topic", "/rear_camera/color/image_raw");
        this->declare_parameter("bbx3d_tolerance", 0.4f);
        this->declare_parameter("working_frame", "camera_link");
        this->declare_parameter("transform_frame", "base_link");
        this->declare_parameter("ground_detection_threshold", 0.3f);
        this->declare_parameter("minimum_probability", 0.3f);

        this->configure();

        pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 1, std::bind(&Darknet3D::pointCloudCb, this, std::placeholders::_1));

        darknet_ros_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            input_bbx_topic_, 1, std::bind(&Darknet3D::darknetCb, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 1, std::bind(&Darknet3D::infoCb, this, std::placeholders::_1));

        camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_image_topic_, 1, std::bind(&Darknet3D::cameraCb, this, std::placeholders::_1));

        raw_camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            raw_camera_topic_, 1, std::bind(&Darknet3D::rawCb, this, std::placeholders::_1));

        darknet3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
            output_bbx3d_topic_, 100);

        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            output_markers_topic_, 1);

        view_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_view_points_topic_, 1);

        human_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_human_points_topic_, 1);

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
    Darknet3D::infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_ = *msg;
        last_detection_ts_ = clock_.now();
    }

    void
    Darknet3D::cameraCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        camera_image_ = *msg;
        last_detection_ts_ = clock_.now();
    }

    void
    Darknet3D::rawCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        raw_image_ = *msg;
        last_detection_ts_ = clock_.now();
    }

    /* 
    * Calculate the lidar points that lie within the field of view of the camera.
    * Publish the filtered point cloud to a new topic viewable in Rviz.
    */
    pcl::PointCloud<pcl::PointXYZ>
    Darknet3D::calculate_view_points(
        pcl::PointCloud<pcl::PointXYZ> cloud,
        image_geometry::PinholeCameraModel cam_model)
    {
        // First we setup a pcl point cloud and do the necessary conversions from a 
        // raw sensor messages point cloud to a PCL object
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(cloud, *new_cloud);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // loop through all points in the cloud to determine which lie in the camera's view
        for (std::size_t i = 0; i < new_cloud->size(); i++) {
            // extract x y and z from the list of points
            int xx = new_cloud->points[i].x;
            int yy = new_cloud->points[i].y;
            int zz = new_cloud->points[i].z;

            // Create a 3d point and project it onto the raw image from the camera sensor
            cv::Point3d point3d;
            // 3d space axis is different from camera projection 3d space
            point3d = cv::Point3d(-yy, -zz, xx);
            // 2d point has x and y that match width and height of camera image
            cv::Point2d point2d = cam_model.project3dToPixel(point3d);

            // get the width and height of the image from the CameraInfo topic
            int camera_width = camera_info_.width;
            int camera_height = camera_info_.height;
            // Check if the project 3d point onto pixels is within the darknet bounding box
            // Also check if points are infront of us 
            if (point2d.x > 0 && point2d.x < camera_width && point2d.y > 0 && point2d.y < camera_height && xx > 0) {
                // We know the point is in our field of view
                if (std::isnan(xx)) {
                    continue;
                }           
            } else {
                // Add the points not in the view to be removed
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

        // return the new cloud containing the points in the view
        return *new_cloud;
    }

    /*
    * Augment a point cloud's points with RGB color values from the camera image.
    * Uses a camera model to correlate a point cloud point to a camera pixel.
    */
    pcl::PointCloud<pcl::PointXYZRGB>
    Darknet3D::augment_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud,
        gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg,
        image_geometry::PinholeCameraModel cam_model) {

        // convert the sensor message image to a opencv image
        cv_bridge::CvImagePtr cv_image;
        cv_image = cv_bridge::toCvCopy(raw_image_, raw_image_.encoding);

        // Further reduce the points on the human by only keeping the points
        // that lie within the new bounding box
        pcl::PointIndices::Ptr inliers_human(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_human;
        for (std::size_t i = 0; i < new_cloud->size(); i++) {
            // Extract the xyz values from the point
            float xx = new_cloud->points[i].x;
            float yy = new_cloud->points[i].y;
            float zz = new_cloud->points[i].z;

            // Check if the point lies within the 3d bounding box
            if (xx >= bbx_msg.xmin-bbx3d_tolerance_ && xx <= bbx_msg.xmax+bbx3d_tolerance_ &&
                yy >= bbx_msg.ymin-bbx3d_tolerance_ && yy <= bbx_msg.ymax+bbx3d_tolerance_ &&
                zz >= bbx_msg.zmin-bbx3d_tolerance_ && zz <= bbx_msg.zmax+bbx3d_tolerance_ ) {

                // edit the point in the point cloud to have the rgb pixel 
                // value of the image
                // Project 3d lidar point to 2d pixel point on the image
                cv::Point3d point3d;
                point3d = cv::Point3d(-yy, -zz, xx);
                cv::Point2d point2d = cam_model.project3dToPixel(point3d);

                // get the BGR values of the pixel
                int b = cv_image->image.at<cv::Vec3b>(point2d.y, point2d.x)[0];
                int g = cv_image->image.at<cv::Vec3b>(point2d.y, point2d.x)[1];
                int r = cv_image->image.at<cv::Vec3b>(point2d.y, point2d.x)[2];
                
                // Assign the color values to the point cloud point
                new_cloud->points[i].r = r;
                new_cloud->points[i].g = g;
                new_cloud->points[i].b = b;
                
            } else {
                // Remove the point not on the human 
                inliers_human->indices.push_back(i);
            }
        }

        // filter the cloud to only include the points on the person
        // Second filter of the point cloud displaying human
        extract_human.setInputCloud(new_cloud);
        extract_human.setIndices(inliers_human);
        extract_human.setNegative(true);
        extract_human.filter(*new_cloud);

        return *new_cloud;
    }

    /*
    * Calculate the 3d bounding boxes that a detected person belongs to
    * Publishes a messages containing lidar points that lie on the human
    * and the camera field of view. 
    */
    void
    Darknet3D::calculate_boxes(
        sensor_msgs::msg::PointCloud2 cloud_pc2,
        pcl::PointCloud<pcl::PointXYZ> cloud,
        gb_visual_detection_3d_msgs::msg::BoundingBoxes3d *boxes,
        float ground_z)
    {
        boxes->header.stamp = cloud_pc2.header.stamp;
        boxes->header.frame_id = cloud_pc2.header.frame_id;

        // Create camera model object and load from the camera info topic
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camera_info_);

        // List to hold the points clouds that contain all the people in the frame
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud_list;
        pcl::PointCloud<pcl::PointXYZ> points_in_view = calculate_view_points(cloud, cam_model);

        // Loop through the bounding boxes
        for (auto bbx : original_bboxes_)
        {
            // Center of darknet bounding box
            int center_x = (bbx.xmax + bbx.xmin) / 2;
            int center_y = (bbx.ymax + bbx.ymin) / 2;

            // Project the center of the darknet bounding box to a unit vector in 3d space
            cv::Point3d center;
            center = cam_model.projectPixelTo3dRay(cv::Point2d(center_x, center_y));

            // setup point indices to extract only the points on the human
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(points_in_view, *new_cloud);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;

            // Ignore bounding boxes with lower probabilities and all other classes
            if (bbx.probability < minimum_probability_ || bbx.class_id == "dont_show")
            {
                // RCLCPP_INFO(this->get_logger(), "Skipped bounding box");
                continue;
            }

            // Setup maximum values for 3d boudning box
            float maxx, minx, maxy, miny, maxz, minz;
            maxx = maxy = maxz = -std::numeric_limits<float>::max();
            minx = miny = minz = std::numeric_limits<float>::max();

            // RCLCPP_INFO(this->get_logger(), "Start point loop");

            // Loop through all points in the point cloud
            for (std::size_t i = 0; i < new_cloud->size(); i++) {
                // Extract the xyz values from the point
                float xx = new_cloud->points[i].x;
                float yy = new_cloud->points[i].y;
                float zz = new_cloud->points[i].z;
                
                // Skip check
                if (bbx.xmin == 0 || bbx.xmax == 0) {
                    RCLCPP_INFO(this->get_logger(), "xmin or xmax had value of 0, skipping");
                    continue;
                }
                // Save time by ignore lidar points on the ground
                if (zz < -ground_z + ground_detection_threshold_) {
                    // Remove the point not on the human 
                    inliers->indices.push_back(i);
                    continue;
                }

                // Project 3d lidar point to 2d pixel point on the image
                cv::Point3d point3d;
                point3d = cv::Point3d(-yy, -zz, xx);
                cv::Point2d point2d = cam_model.project3dToPixel(point3d);

                // RCLCPP_INFO(this->get_logger(), "Pixel X %s Pixel Y %s",
                //     std::to_string(point2d.x).c_str(), std::to_string(point2d.y).c_str());

                // Check if the project lidar point is within the darknet bounding box
                if (point2d.x >= bbx.xmin && point2d.x <= bbx.xmax && 
                    point2d.y >= bbx.ymin && point2d.y <= bbx.ymax) {

                    // Save the max and min values of all the lidar points
                    maxx = std::max(xx, maxx);
                    maxy = std::max(yy, maxy);
                    maxz = std::max(zz, maxz);
                    minx = std::min(xx, minx);
                    miny = std::min(yy, miny);
                    minz = std::min(zz, minz);
                } else {
                    // Remove the point not on the human 
                    inliers->indices.push_back(i);
                }
            }

            // Create the custom 3d bounding box message
            gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg;
            bbx_msg.object_name = bbx.class_id;
            bbx_msg.probability = bbx.probability;

            // Scale the projected 3d unit vector to the location of the person
            float scale = minx/center.z;
            // Dynamically set the bounding box 
            bbx_msg.xmin = center.z * scale - (1 / scale);            
            bbx_msg.xmax = center.z * scale + (1 / scale);            
            bbx_msg.ymin = (-center.x) * scale - (1 / scale);           
            bbx_msg.ymax = (-center.x) * scale + (1 / scale);           
            bbx_msg.zmin = (-center.y) * scale - (1 - (1 / scale));            
            bbx_msg.zmax = (-center.y) * scale + (1 - (1 / scale));

            boxes->bounding_boxes.push_back(bbx_msg);

            // filter the cloud to only include the points on the person
            extract.setInputCloud(new_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*new_cloud);

            // Augment the point cloud with RGB color values from the camera.
            pcl::PointCloud<pcl::PointXYZRGB> human_points;
            human_points = augment_pointcloud(new_cloud, bbx_msg, cam_model);

            point_cloud_list.push_back(human_points);
        }

        // For every bounding box, add the points inside it to a overall point cloud
        pcl::PointCloud<pcl::PointXYZRGB> result_cloud;
        if (point_cloud_list.size() != 0) {
            result_cloud = point_cloud_list[0];
            for (int i = 1; i < point_cloud_list.size(); i++) {
                // concat the point clouds
                result_cloud += point_cloud_list[i];
            }
        }

        // convert all the points in view to a raw sensor messages point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(result_cloud, pcl_pc2);
        sensor_msgs::msg::PointCloud2 raw_pointcloud;
        pcl_conversions::fromPCL(pcl_pc2, raw_pointcloud);

        // publish the point cloud to the ros topic
        if (human_points_pub_->is_activated())
        {
            human_points_pub_->publish(raw_pointcloud);
        }
    }

    /*
    * Publishes the 3d bounding box markers with the appropriate information.
    */
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
            bbx_marker.lifetime = rclcpp::Duration::from_seconds(1.0); //
            bbx_marker.text = bb.object_name;

            msg.markers.push_back(bbx_marker);
        }

        if (markers_pub_->is_activated())
        {
            markers_pub_->publish(msg);
        }
    }

    /*
    * Update gets called whenever there is a new detection of a ros message
    * Most of the functionality lies within this function. 
    */
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
            std::string frame_id = point_cloud_.header.frame_id;
            // std::string frame_id = point_cloud_.header.frame_id.substr(1, -1);
            camera_transform = tfBuffer_.lookupTransform(working_frame_, frame_id,
                                                  point_cloud_.header.stamp, tf2::durationFromSec(2.0));
            base_transform = tfBuffer_.lookupTransform(transform_frame_, working_frame_, 
                                                point_cloud_.header.stamp, tf2::durationFromSec(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
                         ex.what(), "quitting callback");
            return;
        }
        // Transform the point cloud message origin to the camera 
        tf2::doTransform<sensor_msgs::msg::PointCloud2>(point_cloud_, local_pointcloud, camera_transform);

        // Convert the raw point cloud message to a PCL point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(point_cloud_, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        
        // Extract the z value from the base tf frame
        float ground_z = base_transform.transform.translation.z;

        calculate_boxes(local_pointcloud, *cloud, &msg, ground_z);
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
        this->get_parameter("output_markers_topic", output_markers_topic_);
        this->get_parameter("output_view_points_topic", output_view_points_topic_);
        this->get_parameter("output_human_points_topic", output_human_points_topic_);
        this->get_parameter("point_cloud_topic", pointcloud_topic_);
        this->get_parameter("camera_info_topic", camera_info_topic_);
        this->get_parameter("camera_image_topic", camera_image_topic_);
        this->get_parameter("raw_camera_topic", raw_camera_topic_);
        this->get_parameter("bbx3d_tolerance", bbx3d_tolerance_);
        this->get_parameter("working_frame", working_frame_);
        this->get_parameter("transform_frame", transform_frame_);
        this->get_parameter("ground_detection_threshold", ground_detection_threshold_);
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