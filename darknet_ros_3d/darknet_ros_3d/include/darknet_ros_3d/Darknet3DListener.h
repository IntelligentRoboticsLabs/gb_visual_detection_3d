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

#ifndef DARKNET_ROS_3D_DARKNET3DLISTENER_H
#define DARKNET_ROS_3D_DARKNET3DLISTENER_H

#include <ros/ros.h>

#include <darknet_ros_3d_msgs/BoundingBoxes3d.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <list>
#include <vector>

namespace darknet_ros_3d
{

struct DetectedObject
{
  tf2::Vector3 central_point;
  std::string class_id;
  float probability;
  double size_x;
  double size_y;
  double size_z;
};


class Darknet3DListener
{
public:
  Darknet3DListener(const std::list<std::string>& classes, const std::string& working_frame);

  void reset();
  void objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr& msg);

  const std::vector<DetectedObject>& get_objects() {return objects_;}

  void set_working_frame(const std::string& working_frame) {working_frame_ = working_frame;}
  std::string get_working_frame() {return working_frame_;}

  const std::list<std::string>& get_classes() {return classes_;}
  void set_classes(const std::list<std::string>& classes);

  void set_active() {active_ = true;}
  void set_inactive() {active_ = false;}

  void filter_objects(const std::string& class_id, float min_probability,
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    double min_sizex, double max_sizex, double min_sizey, double max_sizey,
    double min_sizez, double max_sizez);

  void print();

private:
  bool is_already_detected(const DetectedObject& object);
  bool is_interested_class(const std::string& class_id);

  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<DetectedObject> objects_;
  std::list<std::string> classes_;
  std::string working_frame_;

  bool active_;
};

};  // namespace darknet_ros_3d

#endif  // DARKNET_ROS_3D_DARKNET3DLISTENER_H
