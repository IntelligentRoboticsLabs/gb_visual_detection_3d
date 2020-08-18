# Package Summary

[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/gb_visual_detection_3d.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/gb_visual_detection_3d)

## darknet_ros_3d

* **Maintainer status:** maintained
* **Maintainer:** Fernando González Ramos <fergonzaramos@yahoo.es>
* **Author:** Francisco Martín Rico <fmrico@gmail.com>; Fernando González Ramos <fergonzaramos@yahoo.es>
* **License:** BSD
* **Source:** [Github](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d)

### Contents
* Overview
  * Previous steps
* Quick start
  * Instalation
  * How it works
* Nodes
  * darknet3d_node

## Overview
darknet_ros_3d provides you 3d bounding boxes of the objects contained in an objects list, where is specificated the 3d position of each object.
Using a RGBD camera like *Asus Xtion*, *Orbbec Astra* or *Intel Realsense* and neuronal network *darknet ros*, objects can be detected and his position can be calculated.

In addition, there is a visual debugger tool based on visual markers that you can see with rviz

![Image](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d/raw/master/docs/visual_markers.png)

### Previous Steps

The first we have to know is that *darknet_ros_3d* package have dependencies as follow:

* rclcpp
* rclcpp_lifecycle
* darknet_ros_msgs
* gb_visual_detection_3d_msgs
* sensor_msgs
* tf2
* tf2_ros
* tf2_sensor_msgs
* visualization_msgs
* eigen3_cmake_module
* Eigen3

**You can install Darknet Ros following** [this steps](https://github.com/leggedrobotics/darknet_ros). **NOTE:** ros2 branch

## Quick Start

### Instalation

You must to clone *gb_visual_detection_3d* into *src* folder located in your workspace. Later, you have to compile it typing ``colcon build``.

### How It Works

First of all, is necessary to run camera driver. **Make sure that camera driver is publishing point cloud information**.

Later, you must run *darknet_ros* and, if everything worked properly, you should see 2d bounding boxes in your screen. If not, you have a problem with *darknet_ros* package.

Now, you can run *darknet_ros_3d* typing ``ros2 launch darknet_ros_3d darknet_ros_3d.launch.py``. If you want to change default parameters like topics it subscribe, you can change it in the configuration file located at ``~/catkin_ws/src/gb_visual_detection_3d/darknet_ros_3d/darknet_ros_3d/config/``. Default parameters are the following:

* **interested_classes:** Classes you want to detect. It must be classes names than exists previously in darknet ros.

* **mininum_detection_threshold:** Maximum distance range between any pixel of image and the center pixel of the image to be considered.

* **minimum_probability:** Minimum object probability (provided by *darknet_ros*) to be considered.

* **darknet_ros_topic:** topic where darknet_ros publicates it's bounding boxes. ``/darknet_ros/bounding_boxes``.

* **point_cloud_topic:** topic where point cloud is published from camera. By default: ``/camera/pointcloud``. **It is important that point cloud topic be of PointCloud2 type and it be depth_registered**.

* **working_frame:** frame that all measurements are based on. By default, *camera_link*. **It is very important that if you want to change this frame, it has the same axes than camera_link**, if you would want 3d coordinates in another axis, you must change it later (once 3d bounding box has been calculated).

## Nodes

### darknet3d_node

*darknet3d_node* provide bounding boxes. This bounding boxes are combinated with point cloud information to calculate (xmin, ymin, zmin) and (xmax, ymax, zmax) 3D coordinates.

Then, *darknet_ros_3d* publicates it's own bounding boxes array of *BoundingBoxes3d* type in ``/darknet_ros_3d/bounding_boxes_3d`` by default, which is an array of *BoundingBox3d* that contains the following information:
```
std_msgs/Header header
BoundingBox3d[] bounding_boxes
```

**BoundingBox3d:**

```
string object_name
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax
```
* **object_name:** Object name.
* **probability:** Probability of certainty.
* **xmin:** X coordinate in meters of left upper corner of bounding box.
* **xmax:** X coordinate in meters of right lower corner of bounding box.
* **ymin:** Y coordinate in meters of left upper corner of bounding box.
* **ymax:** Y coordinate in meters of right lower corner of bounding box.
* **zmin:** Z coordinate in meters of nearest pixel of bounding box.
* **zmax:** Z coordinate in meters of the furthest pixel of bounding box.

**An example of output is as follow:**

```
header:
  stamp:
    sec: 1593723845
    nanosec: 430724839
  frame_id: camera_link
bounding_boxes:
- object_name: person
  probability: 0.7609682679176331
  xmin: 0.4506256580352783
  ymin: -0.3164764642715454
  xmax: 0.7936256527900696
  ymax: 0.11368180811405182
  zmin: -0.25958430767059326
  zmax: 0.10506562888622284

```

You can visualize the markers on *rviz* adding **MarkerArray** and subscribing to topic ``/darknet_ros_3d/markers``.
