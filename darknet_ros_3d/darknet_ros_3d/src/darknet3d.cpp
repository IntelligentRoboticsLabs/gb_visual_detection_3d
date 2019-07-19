#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_3d_msgs/BoundingBox3d.h"
#include "darknet_ros_3d_msgs/BoundingBoxes3d.h"
#include "sensor_msgs/Image.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include "std_msgs/Int32.h"
#include <mutex>
#include <bica/Component.h>

class Darknet3d: public bica::Component
{
private:
  ros::NodeHandle n;
  /*
  *yolo_sub: Subscriber to bounding boxes provided by darknet_ros
  *depth_sub: subscriber to depth image provided by openni2_launch package
  */
  ros::Subscriber _yolo_sub, _depth_sub;
  ros::Publisher _darknet3d_pub;

  std::vector<darknet_ros_msgs::BoundingBox> _originalBBoxes; //here is saved bounding boxes array received from darknet_ros
  float _distRange, _minCameraDist, _maxCameraDist; //max difference between nearest pixel and any other pixel.
  std::mutex _mtx;

public:
  Darknet3d()
  {
    initParams();
    _darknet3d_pub = n.advertise<darknet_ros_3d_msgs::BoundingBoxes3d>("/darknet_ros_3d/bounding_boxes", 1);
  }

  void initParams()
  {
    std::string s;
    //set darknet ros topic to subscribe
    if(! n.getParam("darknet_ros_topic", s))
    {
      s = "/darknet_ros/bounding_boxes";
    }
    ROS_INFO("Darknet topic: %s", s.c_str());
    _yolo_sub = n.subscribe(s, 1, &Darknet3d::darknetCb, this);

    //set depth image topic to subscribe
    if(! n.getParam("depth_image_topic", s))
    {
      s = "/camera/depth/image_rawNuevo";
    }
    ROS_INFO("Depth topic: %s", s.c_str());
    _depth_sub = n.subscribe(s, 1, &Darknet3d::depthCb, this);

    float f;
    //set maximum distance range between pixels
    if(! n.getParam("dist_range_pixels", f))
    {
      f = 1000.0;
    }
    ROS_INFO("Dist range: %f", f);
    _distRange = f;

    //set min distance camera is be able to detect
    if(! n.getParam("min_camera_dist", f))
    {
      f = 500.0;
    }
    ROS_INFO("Min camera Dist: %f", f);
    _minCameraDist = f;
  }

  float mean(std::vector<float> v)
  {
    float sum;
    sum = 0.0;
    for(int i = 0; i < v.size(); i++)
    {
      sum = sum + v[i];
    }
    return sum / (float)v.size();
  }

  float flood(float nearestPixel, cv::Mat croppedImage)
  {
    float dist;

    std::vector<float> distsArray;

    for(int j = 0; j < croppedImage.size().height; j++)
    {
      for(int k = 0; k < croppedImage.size().width; k++)
      {
        dist = (float)croppedImage.at<float>(j, k);
        //ROS_INFO("Traza flood bucle");
        if((dist - nearestPixel) <= _distRange)
        {
          distsArray.push_back(dist);
        }
      }
    }
    return Darknet3d::mean(distsArray);

  }

  float getMinDist(cv::Mat croppedImage)
  {
    //ROS_INFO("Traza MinDist once");
    float dist, mindist;

    mindist = (float)croppedImage.at<float>(0, 0);
    for(int j = 0; j < croppedImage.size().height; j++)
    {

      for(int k = 0; k < croppedImage.size().width; k++)
      {
        //ROS_INFO("Traza MinDist bucle encima");
        dist = (float)croppedImage.at<float>(j, k);
      //  ROS_INFO("dist: %f", dist);
        if((dist > _minCameraDist) && (dist < mindist))
        {
          mindist = dist;
        }
      }
    }

    if(mindist < _minCameraDist)
    {
      mindist = _minCameraDist;
    }

    return mindist;
  }

  float getDist(darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg)
  {
    float nearestPixel; //Distance to the nearest pixel is saved here
    int width, height;
    cv::Mat croppedImage;

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    width = p.xmax - p.xmin;
    height = p.ymax - p.ymin;
    cv::Mat ROI(cv_image->image, cv::Rect(p.xmin, p.ymin,
                width, height));
    ROI.copyTo(croppedImage);
    //Get nearest pixel
    nearestPixel = Darknet3d::getMinDist(croppedImage);

    //Algorithm to disregard very distant pixels and calculate mean distance
    return Darknet3d::flood(nearestPixel, croppedImage);
  }

  void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    if(isActive())
    {
      _mtx.lock();
      _originalBBoxes = msg->bounding_boxes;
      _mtx.unlock();
    }
  }

  void depthCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    if(isActive())
    {
      std::vector<darknet_ros_3d_msgs::BoundingBox3d> v;
      darknet_ros_3d_msgs::BoundingBox3d data;
      std::vector<darknet_ros_msgs::BoundingBox> originalBBoxes;

      _mtx.lock();
      originalBBoxes = _originalBBoxes;
      _mtx.unlock();

      if(originalBBoxes.size() > 0)
      {
        float dist;
        //_mtx.lock();
        for(int i = 0; i < originalBBoxes.size(); i++)
        {
          //Componer el mensaje
          data.Class = originalBBoxes[i].Class;
          data.probability = originalBBoxes[i].probability;
          data.xmin = originalBBoxes[i].xmin;
          data.ymin = originalBBoxes[i].ymin;
          data.xmax = originalBBoxes[i].xmax;
          data.ymax = originalBBoxes[i].ymax;

          dist = Darknet3d::getDist(originalBBoxes[i], msg);
          data.depth = dist;
          //meterlo en el array
          v.push_back(data);
        }
        //Publicar el array
        darknet_ros_3d_msgs::BoundingBoxes3d msg_to_publish;
        msg_to_publish.bounding_boxes = v;
        _darknet3d_pub.publish(msg_to_publish);
      }
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darknet_3d");
  Darknet3d darknet3d;
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::spin();
  return 0;
}
