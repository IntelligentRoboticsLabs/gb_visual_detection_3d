# Darknet Ros 3d. Usage Guide


[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/gb_visual_detection_3d.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/gb_visual_detection_3d)


## About Darknet Ros 3d
Darknet Ros 3d allows you to know all bounding boxes that Darknet Ros also provide you but adding the stimated distance to each bounding box. Later, how it works is explained.

## Previous Steps

The first we have to know is that *darknet_ros_3d* package have dependencies as follow:

* darknet_ros_msgs
* darknet_ros_3d_msgs
* std_msgs
* cv_bridge
* roscpp
* rospy

**You can install Darknet Ros following** [this steps](https://github.com/leggedrobotics/darknet_ros)
## Instalation

You must to clone *darknet_ros_3d* into *src* folder located in your catkin workspace. Later, you have to compile it typing ``catkin_make``. If it doesn't work, it's possible that you must compile typing ``catkin_make -j1``.

## How It Works

First of all, is necessary to run camera driver and darknet ros.

For xtion RGBD camera, you have to install *openni2_launch* package and then, type ``roslaunch openni2_launch openni2.launch``. If you have any other RGBD camera, you have to launch his driver.

To run darknet ros, type ``roslaunch darknet_ros darknet_ros.launch``.

Now, you can run *darknet_ros_3d* typing ``roslaunch darknet_ros_3d darknet_ros_3d.launch``.


## Nodes

### darknet3d_node

### Subscribed Topics

-
