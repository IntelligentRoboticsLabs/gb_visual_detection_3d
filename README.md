# Darknet Ros 3d. Usage Guide

## About Darknet Ros 3d
Darknet Ros 3d allows you to know all bounding boxes that Darknet Ros also provide you but adding the stimated distance to each bounding box. Later, how it works is explained.

## Previous Steps

The first we have to know is that *darknet_ros_3d* package have dependencies as follow:

* darknet_ros_msgs
* darknet_ros_3d_msgs
* std_msgs
* cv_bridge
* bica
* bica_msgs
* roscpp
* rospy

**You can install Darknet Ros following** [this steps](https://github.com/leggedrobotics/darknet_ros) **and BICA following** [this steps](https://github.com/IntelligentRoboticsLabs/BICA).

## Instalation

You must to clone *darknet_ros_3d* into *src* folder located in your catkin workspace. Later, you have to compile it typing ``catkin_make``. If it doesn't work, it's possible that you must compile typing ``catkin_make -j1``.

## How It Works

First of all, is necessary to run camera driver and darknet ros.

For xtion RGBD camera, you have to install *openni2_launch* package and then, type ``roslaunch openni2_launch openni2.launch``. If you have any other RGBD camera, you have to launch his driver.

To run darknet ros, type ``roslaunch darknet_ros darknet_ros.launch``.

Now, you can run *darknet_ros_3d* typing ``roslaunch darknet_ros_3d darknet_ros_3d.launch``. If you want to change default parameters like topics it subscribe, you can change it in the launch file. Default parameters are the following:

* **darknet_ros_topic:** topic where darknet ros publicates his bounding boxes. ``/darknet_ros/bounding_boxes``

* **depth_image_topic:** topic where camera driver publicates the depth image. ``/camera/depth/image_raw``

* **dist_range_pixels:** max distance between nearest pixel and any other pixel of image to be countered.

* **min_camera_dist:** minimum distance that camera is be able to detect.

To conclude, the topic where darknet_ros_3d publicates his output is ``/darknet_ros_3d/bounding_boxes``.

**NOTE:** *Node that runs launch file is a BICA component*.
