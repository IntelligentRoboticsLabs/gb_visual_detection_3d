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
#include <algorithm>

namespace darknet_ros_3d
{

Darknet3DListener::Darknet3DListener(const std::string& working_frame):
  nh_(),
  tf_listener_(tfBuffer_),
  working_frame_(working_frame),
  active_(false)
{
  object_sub_ = nh_.subscribe("/darknet_ros_3d/bounding_boxes", 10, &Darknet3DListener::objectsCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.5), &Darknet3DListener::timer_callback, this);
}

void
Darknet3DListener::reset()
{
  objects_.clear();
}

bool
Darknet3DListener::is_interested_class(const std::string& class_id)
{
  return classes_conf_.find(class_id) != classes_conf_.end();
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
Darknet3DListener::add_class(const std::string& class_id, const ObjectConfiguration& conf)
{
  classes_conf_[class_id] = conf;
}

bool
Darknet3DListener::is_valid_object(const DetectedObject& object)
{
  const ObjectConfiguration& conf = classes_conf_[object.class_id];

  /*ROS_INFO("Conf: [%s]", object.class_id.c_str());
  ROS_INFO("\tmin_probability = %lf", conf.min_probability);
  ROS_INFO("\tX = [%lf, %lf]", conf.min_x, conf.max_x);
  ROS_INFO("\tY = [%lf, %lf]", conf.min_y, conf.max_y);
  ROS_INFO("\tZ = [%lf, %lf]", conf.min_z, conf.max_z);
  ROS_INFO("\tSize X = [%lf, %lf]", conf.min_size_x, conf.max_size_x);
  ROS_INFO("\tSize Y = [%lf, %lf]", conf.min_size_y, conf.max_size_y);
  ROS_INFO("\tSize Z = [%lf, %lf]", conf.min_size_z, conf.max_size_z);
*/
  /*if (object.probability <= conf.min_probability) ROS_INFO("Excluded by Probability");
  if (object.central_point.x() <= conf.min_x || object.central_point.x() >= conf.max_x) ROS_INFO("Excluded by Pos X");
  if (object.central_point.y() <= conf.min_y || object.central_point.y() >= conf.max_y) ROS_INFO("Excluded by Pos Y");
  if (object.central_point.z() <= conf.min_z || object.central_point.z() >= conf.max_z) ROS_INFO("Excluded by Pos Z");
  if (object.size_x <= conf.min_size_x || object.size_x >= conf.max_size_x) ROS_INFO("Excluded by Size X");
  if (object.size_y <= conf.min_size_y || object.size_y >= conf.max_size_y) ROS_INFO("Excluded by Size Y");
  if (object.size_z <= conf.min_size_z || object.size_z >= conf.max_size_z) ROS_INFO("Excluded by Size Z");*/

  return object.probability > conf.min_probability &&
    object.central_point.x() > conf.min_x && object.central_point.x() < conf.max_x &&
    object.central_point.y() > conf.min_y && object.central_point.y() < conf.max_y &&
    object.central_point.z() > conf.min_z && object.central_point.z() < conf.max_z &&
    object.size_x > conf.min_size_x && object.size_x < conf.max_size_x &&
    object.size_y > conf.min_size_y && object.size_y < conf.max_size_y &&
    object.size_z > conf.min_size_z && object.size_z < conf.max_size_z;
}

void
Darknet3DListener::objectsCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
  if (!active_)
    return;

  int counter = 0;
  for (const auto& bb : msg->bounding_boxes)
  {
    // ROS_INFO("Object received [%s]", bb.Class.c_str());

    if (!is_interested_class(bb.Class))
      continue;

    geometry_msgs::TransformStamped any2wf_msg;
    tf2::Transform any2wf;

    std::string error;
    if (tfBuffer_.canTransform(msg->header.frame_id, working_frame_,
       msg->header.stamp, ros::Duration(0.1), &error))
      any2wf_msg = tfBuffer_.lookupTransform(msg->header.frame_id, working_frame_, msg->header.stamp);
    else
    {
      ROS_ERROR("Can't transform %s", error.c_str());
      return;
    }

    tf2::Stamped<tf2::Transform> aux;
    tf2::convert(any2wf_msg, aux);

    any2wf = aux;

    DetectedObject object;

    object.central_point.setX((bb.xmax + bb.xmin)/2.0);
    object.central_point.setY((bb.ymax + bb.ymin)/2.0);
    object.central_point.setZ((bb.zmax + bb.zmin)/2.0);

    object.central_point = any2wf.inverse() * object.central_point;

    object.size_x = bb.xmax - bb.xmin;
    object.size_y = bb.ymax - bb.ymin;
    object.size_z = bb.zmax - bb.zmin;
    object.class_id = bb.Class;
    object.probability = bb.probability;

    /*ROS_INFO("-----------> %d prob=%f  [%s] coords=(%lf %lf, %lf) sizes=[%lf, %lf, %lf]", counter++,
      object.probability, object.class_id.c_str(),
      object.central_point.x(), object.central_point.y(), object.central_point.z(),
      object.size_x, object.size_y, object.size_z);*/

    if (is_valid_object(object))
    {
      // ROS_INFO("Is valid");
      add_object(object);
    }
    else
    {
      // ROS_INFO("Is not valid");
    }
  }

  check_objects_history();
}

void
Darknet3DListener::check_objects_history()
{
  for (auto& object : objects_)
  {
    const ObjectConfiguration& conf = classes_conf_[object.class_id];
    if (conf.dynamic)
    {
      while (!object.history.empty() &&
             (ros::Time::now() - object.history.front().stamp_) > conf.max_seconds)
      {
        // ROS_INFO("Removing because (%lf > %lf)", (ros::Time::now() - object.history.front().stamp_).toSec(),
        //  conf.max_seconds.toSec());
        object.history.pop_front();
      }
    }
  }
}

void
Darknet3DListener::timer_callback(const ros::TimerEvent& ev)
{
  check_objects_history();

  for (auto& object : objects_)
  {
    update_speed(object);
  }

  auto it = objects_.begin();
  while (it != objects_.end())
  {
    const ObjectConfiguration& conf = classes_conf_[it->class_id];
    if (conf.dynamic && it->history.empty())
      it = objects_.erase(it);
    else
      ++it;
  }
}


void
Darknet3DListener::add_object(const DetectedObject& object)
{
  bool new_object = true;
  for (auto& existing_object : objects_)
  {
    if (same_object(existing_object, object))
    {
      // ROS_INFO("Merging with one existing");
      merge_objects(existing_object, object);
      new_object = false;
    }
    else if (!other_object(existing_object, object))
    {
      // ROS_INFO("Not merging, but it is not other object");
      new_object = false;
    }
  }

  if (new_object)
  {
    ROS_INFO("Adding a new object");
    objects_.push_back(object);
  }
}

bool
Darknet3DListener::other_object(const DetectedObject& obj1, const DetectedObject& obj2)
{
  /*if (obj1.class_id == obj2.class_id) ROS_INFO("\tIt is the same class");
  if (fabs(obj1.central_point.x() - obj2.central_point.x()) < (obj1.size_x + obj2.size_x))
    ROS_INFO("\tX (dist %lf < %lf)", fabs(obj1.central_point.x() - obj2.central_point.x()),  (obj1.size_x / 1.8 + obj2.size_x/ 1.8));
  if (fabs(obj1.central_point.y() - obj2.central_point.y()) < (obj1.size_y + obj2.size_y))
    ROS_INFO("\tY (dist %lf < %lf)", fabs(obj1.central_point.y() - obj2.central_point.y()), (obj1.size_y / 1.8 + obj2.size_y / 1.8));
  if (fabs(obj1.central_point.z() - obj2.central_point.z()) < (obj1.size_z + obj2.size_z))
    ROS_INFO("\tZ (dist %lf < %lf)", fabs(obj1.central_point.z() - obj2.central_point.z()), (obj1.size_z / 1.8 + obj2.size_z / 1.8));
    */
  return  obj1.class_id != obj2.class_id ||
          (fabs(obj1.central_point.x() - obj2.central_point.x()) > (obj1.size_x / 1.9 + obj2.size_x / 1.9)) ||
          (fabs(obj1.central_point.y() - obj2.central_point.y()) > (obj1.size_y / 1.9 + obj2.size_y / 1.9)) ||
          (fabs(obj1.central_point.z() - obj2.central_point.z()) > (obj1.size_z / 1.9 + obj2.size_z / 1.9));
}

bool
Darknet3DListener::same_object(const DetectedObject& obj1, const DetectedObject& obj2)
{
  return obj1.class_id == obj2.class_id &&
         (fabs(obj1.central_point.x() - obj2.central_point.x()) < (obj1.size_x / 2.0 + obj2.size_x / 2.0)) &&
         (fabs(obj1.central_point.y() - obj2.central_point.y()) < (obj1.size_y / 2.0 + obj2.size_y / 2.0)) &&
         (fabs(obj1.central_point.z() - obj2.central_point.z()) < (obj1.size_z / 2.0 + obj2.size_z / 2.0));
}

void
Darknet3DListener::merge_objects(DetectedObject& existing_object, const DetectedObject& new_object)
{
  double e_min_x = existing_object.central_point.x() - existing_object.size_x / 2.0;
  double e_min_y = existing_object.central_point.y() - existing_object.size_y / 2.0;
  double e_min_z = existing_object.central_point.z() - existing_object.size_z / 2.0;
  double e_max_x = existing_object.central_point.x() + existing_object.size_x / 2.0;
  double e_max_y = existing_object.central_point.y() + existing_object.size_y / 2.0;
  double e_max_z = existing_object.central_point.z() + existing_object.size_z / 2.0;

  double n_min_x = new_object.central_point.x() - new_object.size_x / 2.0;
  double n_min_y = new_object.central_point.y() - new_object.size_y / 2.0;
  double n_min_z = new_object.central_point.z() - new_object.size_z / 2.0;
  double n_max_x = new_object.central_point.x() + new_object.size_x / 2.0;
  double n_max_y = new_object.central_point.y() + new_object.size_y / 2.0;
  double n_max_z = new_object.central_point.z() + new_object.size_z / 2.0;

  double min_x = std::min(e_min_x, n_min_x);
  double min_y = std::min(e_min_y, n_min_y);
  double min_z = std::min(e_min_z, n_min_z);
  double max_x = std::max(e_max_x, n_max_x);
  double max_y = std::max(e_max_y, n_max_y);
  double max_z = std::max(e_max_z, n_max_z);

  const ObjectConfiguration& conf = classes_conf_[existing_object.class_id];

  double x = (max_x + min_x) / 2.0;
  double y = (max_y + min_y) / 2.0;
  double z = (max_z + min_z) / 2.0;
  x = std::max(std::min(x, conf.max_x), conf.min_x);
  y = std::max(std::min(y, conf.max_y), conf.min_y);
  z = std::max(std::min(z, conf.max_z), conf.min_z);

  if (conf.dynamic)
  {
    tf2::Stamped<tf2::Vector3> point;
    point.stamp_ = ros::Time::now();
    point.frame_id_ = working_frame_;
    point.setX(x);
    point.setY(y);
    point.setZ(z);
    existing_object.history.push_back(point);

    update_speed(existing_object);
  }

  existing_object.central_point.setX(x);
  existing_object.central_point.setY(y);
  existing_object.central_point.setZ(z);

  existing_object.size_x = std::max(std::min(max_x - min_x, conf.max_size_x), conf.min_size_x);
  existing_object.size_y = std::max(std::min(max_y - min_y, conf.max_size_y), conf.min_size_y);
  existing_object.size_z = std::max(std::min(max_z - min_z, conf.max_size_z), conf.min_size_z);

  existing_object.probability = (existing_object.probability + new_object.probability) / 2.0;
}

void
Darknet3DListener::update_speed(DetectedObject& object)
{
  if (object.history.size() > 2)
  {
    double diff_time = (object.history.back().stamp_ - object.history.front().stamp_).toSec();

    // ROS_INFO("Speed calc = (%lf - %lf) / %lf", object.history.back().x(), object.history.front().x(), diff_time);

    double vx = (object.history.back().x() - object.history.front().x()) / diff_time;
    double vy = (object.history.back().y() - object.history.front().y()) / diff_time;
    double vz = (object.history.back().z() - object.history.front().z()) / diff_time;

    if (fabs(vx) < 2.0 && fabs(vy) < 2.0 && fabs(vz) < 2.0)
    {
      object.speed.setX(vx);
      object.speed.setY(vy);
      object.speed.setZ(vz);
    }
  }
  else
  {
    object.speed.setX(0.0);
    object.speed.setY(0.0);
    object.speed.setZ(0.0);
  }
}

void
Darknet3DListener::print()
{
  int counter = 0;
  ROS_INFO("============> Number of ojects %zu", objects_.size());
  for (const auto& test_obj : objects_)
  {
    ROS_INFO("============> %d prob=%f  [%s] coords=(%lf %lf, %lf) sizes=[%lf, %lf, %lf] speed={%lf, %lf, %lf}",
      counter++,
      test_obj.probability, test_obj.class_id.c_str(),
      test_obj.central_point.x(), test_obj.central_point.y(), test_obj.central_point.z(),
      test_obj.size_x, test_obj.size_y, test_obj.size_z,
      test_obj.speed.x(), test_obj.speed.y(), test_obj.speed.z());

      /*const ObjectConfiguration& conf = classes_conf_[test_obj.class_id];
      if (conf.dynamic)
      {
        for (const auto& point : test_obj.history)
        {
          ROS_INFO("\t[%f] (%lf, %lf, %lf)", point.stamp_.toSec(), point.x(), point.y(), point.z());
        }
      }*/
  }
}

};  // namespace darknet_ros_3d
