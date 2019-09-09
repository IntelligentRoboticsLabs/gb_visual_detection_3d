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

#include "darknet_ros_3d/Darknet3DListener.h"

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <list>

namespace darknet_ros_3d
{

Darknet3DListener::Darknet3DListener(const std::list<std::string>& classes, const std::string& working_frame):
  nh_(),
  tf_listener_(tfBuffer_),
  classes_(classes),
  working_frame_(working_frame),
  active_(false)
{
  object_sub_ = nh_.subscribe("/darknet_ros_3d/bounding_boxes", 10, &Darknet3DListener::objectsCallback, this);
}

void
Darknet3DListener::reset()
{
  objects_.clear();
}

bool
Darknet3DListener::is_interested_class(const std::string& class_id)
{
  return std::find(classes_.begin(), classes_.end(), class_id) != classes_.end();
}

bool
Darknet3DListener::is_already_detected(const DetectedObject& object)
{
  for (const auto& test_obj : objects_)
  {
    if ((test_obj.class_id == object.class_id) &&
        (fabs(test_obj.central_point.x() - object.central_point.x()) < (test_obj.size_x / 2.0 + object.size_x / 2.0)) &&
        (fabs(test_obj.central_point.y() - object.central_point.y()) < (test_obj.size_y / 2.0 + object.size_y / 2.0)) &&
        (fabs(test_obj.central_point.z() - object.central_point.z()) < (test_obj.size_z / 2.0 + object.size_z / 2.0)))
    {
      return true;
    }
  }
  return false;
}

void
Darknet3DListener::filter_objects(const std::string& class_id, float min_probability,
  double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
  double min_sizex, double max_sizex, double min_sizey, double max_sizey, double min_sizez, double max_sizez)
{
  auto it = objects_.begin();
  while (it != objects_.end())
  {
    if ((it->class_id == class_id) &&
        (it->probability < min_probability ||
         it->central_point.x() < min_x || it->central_point.x() > max_x ||
         it->central_point.y() < min_y || it->central_point.y() > max_y ||
         it->central_point.z() < min_z || it->central_point.z() > max_z ||
         it->size_x < min_sizex || it->size_x > max_sizex ||
         it->size_y < min_sizey || it->size_y > max_sizey ||
         it->size_z < min_sizez || it->size_z > max_sizez))
      it = objects_.erase(it);
    else
      ++it;
  }
}

void
Darknet3DListener::objectsCallback(const darknet_ros_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
  if (!active_)
    return;

  for (const auto& bb : msg->bounding_boxes)
  {
    if (!is_interested_class(bb.Class))
      continue;

    geometry_msgs::TransformStamped any2wf_msg;
    tf2::Transform any2wf;

    std::string error;
    if (tfBuffer_.canTransform(msg->header.frame_id, working_frame_,
      ros::Time(0), ros::Duration(0.1), &error))
      any2wf_msg = tfBuffer_.lookupTransform(msg->header.frame_id, working_frame_, ros::Time(0));
    else
    {
      ROS_ERROR("Can't transform %s", error.c_str());
      return;
    }

    tf2::Stamped<tf2::Transform> aux;
    tf2::convert(any2wf_msg, aux);

    any2wf = aux;

    DetectedObject point;

    point.central_point.setX((bb.xmax + bb.xmin)/2.0);
    point.central_point.setY((bb.ymax + bb.ymin)/2.0);
    point.central_point.setZ((bb.zmax + bb.zmin)/2.0);

    point.central_point = any2wf.inverse() * point.central_point;

    point.size_x = bb.xmax - bb.xmin;
    point.size_y = bb.ymax - bb.ymin;
    point.size_z = bb.zmax - bb.zmin;

    point.class_id = bb.Class;
    point.probability = bb.probability;

    if (!is_already_detected(point))
      objects_.push_back(point);

  }
}

void
Darknet3DListener::print()
{
  int counter = 0;
  ROS_INFO("============> Number of ojects %zu", objects_.size());
  for (const auto& test_obj : objects_)
  {
    ROS_INFO("============> %d [%s] (%lf %lf, %lf) [%lf, %lf, %lf]", counter++, test_obj.class_id.c_str(),
      test_obj.central_point.x(), test_obj.central_point.y(), test_obj.central_point.z(),
      test_obj.size_x, test_obj.size_y, test_obj.size_z);
  }
}

};  // namespace darknet_ros_3d
