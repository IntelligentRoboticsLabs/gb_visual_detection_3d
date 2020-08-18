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
Using a RGBD camera like Asus Xtion and neuronal network *darknet ros*, objects can be detected and his position can be calculated.

In addition, there is a visual debugger tool based on visual markers that you can see with rviz

![Image](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d/raw/master/docs/visual_markers.png)
![Image](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d/raw/master/docs/visual_markers2.png)

### Previous Steps

The first we have to know is that *darknet_ros_3d* package have dependencies as follow:

* roscpp
* darknet_ros_msgs
* [gb_visual_detection_3d_msgs](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs)
* sensor_msgs
* tf
* pcl_ros
* pcl_conversions
* roslint

**You can install Darknet Ros following** [this steps](https://github.com/leggedrobotics/darknet_ros)

## Quick Start

### Instalation

You must to clone *gb_visual_detection_3d* into *src* folder located in your catkin workspace. Later, you have to compile it typing ``catkin_make``. If it doesn't work, it's possible that you must compile typing ``catkin_make -j1``.

### How It Works

First of all, is necessary to run camera driver.

For xtion RGBD camera, you have to install *openni2_launch* package and then, type ``roslaunch openni2_launch openni2.launch``. If you have any other RGBD camera, like Intel RealSense, you have to launch his driver.

Now, you can run *darknet_ros_3d* typing ``roslaunch darknet_ros_3d darknet_ros_3d.launch``. If you want to change default parameters like topics it subscribe, you can change it in the configuration file located at ``~/catkin_ws/src/gb_visual_detection_3d/darknet_ros_3d/darknet_ros_3d/config/``. Default parameters are the following:

* **interested_classes:** Classes you want to detect. It must be classes names than exists previously in darknet ros.

* **mininum_detection_threshold:** Maximum distance range between any pixel of image and the center pixel of the image to be considered.

* **minimum_probability:** Minimum object probability (provided by *darknet_ros*) to be considered.

* **darknet_ros_topic:** topic where darknet ros publicates his bounding boxes. ``/darknet_ros/bounding_boxes``

* **point_cloud_topic:** topic where point cloud is published from camera. By default: ``/camera/depth_registered/points``. **It is important that point cloud topic be of PointCloud2 type and it be depth_registered**

* **working_frame:** frame that all measurements are based on. By default, *camera_link*.

**NOTE:** color image topic that darknet_ros needs, can be edited in the launch file retyping *camera_rgb_topic* argument.

## Nodes

### darknet3d_node

*darknet3d_node* provide bounding boxes. This bounding boxes are combinated with point cloud information to calculate (xmin, ymin, zmin) and (xmax, ymax, zmax) 3D coordinates.

Then, *darknet_ros_3d* publicates his own bounding boxes array of *BoundingBoxes3d* type, which is an array of *BoundingBox3d* that contains the following information:
```
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax
```
* **Class:** Object name.
* **probability:** Probability of certainty.
* **xmin:** X coordinate in meters of left upper corner of bounding box.
* **xmax:** X coordinate in meters of right lower corner of bounding box.
* **ymin:** Y coordinate in meters of left upper corner of bounding box.
* **ymax:** Y coordinate in meters of right lower corner of bounding box.
* **zmin:** Z coordinate in meters of nearest pixel of bounding box.
* **zmax:** Z coordinate in meters of the furthest pixel of bounding box.

**An example of output is as follow:**

```
bounding_boxes:
  -
    Class: "person"
    probability: 0.805726051331
    xmin: 0.462000012398
    ymin: -0.340014994144
    xmax: 0.851000010967
    ymax: 0.1600689888
    zmin: -0.267071247101
    zmax: 0.186251342297
```

You can visualize the markers on *rviz* adding **MarkerArray** and subscribing to topic ``/darknet_ros_3d/markers``.
