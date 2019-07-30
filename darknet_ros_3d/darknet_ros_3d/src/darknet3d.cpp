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
#include "visualization_msgs/Marker.h"
#include <mutex>
#include <bica/Component.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/CameraInfo.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

const float Tolerance = 0.02;

class Darknet3d: public bica::Component
{
private:
  struct Type_Coord{
    int xcoor;
    int ycoor;
  };
  ros::NodeHandle n;
  /*
  *yolo_sub: Subscriber to bounding boxes provided by darknet_ros
  *depth_sub: subscriber to depth image provided by openni2_launch package
  */
  ros::Subscriber _yolo_sub, _depth_sub, _pointCloud_sub, _cameraInfo_sub;
  ros::Publisher _darknet3d_pub, _marker_pub;

  std::vector<darknet_ros_msgs::BoundingBox> _originalBBoxes; //here is saved bounding boxes array received from darknet_ros
  float _distRange, _minCameraDist, _maxCameraDist; //max difference between nearest pixel and any other pixel.
  int _cameraHeight, _cameraWidth; //Camera resolution
  bool _arrived;
  sensor_msgs::PointCloud _point_cloud;

public:
  Darknet3d()
  {
    initParams();
    _pointCloud_sub = n.subscribe("/camera/depth_registered/points", 1, &Darknet3d::pointCloudCb, this);
    _cameraInfo_sub = n.subscribe("/camera/rgb/camera_info", 1, &Darknet3d::cameraInfoCb, this);
    _darknet3d_pub = n.advertise<darknet_ros_3d_msgs::BoundingBoxes3d>("/darknet_ros_3d/bounding_boxes", 1);
    _marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

    _arrived = false;
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
    _depth_sub = n.subscribe(s, 1, &Darknet3d::pointCloudCb, this);

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

  float getMax(std::vector<float> v)
  {
    float maxDist = 0;
    for(int i = 0; i < v.size(); i++)
    {
      if(v[i] > maxDist){
        maxDist = v[i];
      }
    }
    return maxDist;
  }

  void lookColumn(std::vector<float> v, int column, cv::Mat croppedImage)
  {
    float dist, distPrev;
    Type_Coord centerPixel;
    centerPixel.ycoor = int(croppedImage.size().height / 2);
    int j;
    bool finish;
    //Miro por debajo
    distPrev = (float)croppedImage.at<float>(centerPixel.ycoor, column);

    j = centerPixel.ycoor + 1;
    finish = false;
    while(! finish && j < croppedImage.size().height)
    {
      dist = (float)croppedImage.at<float>(j, column);
      if(abs(dist - distPrev) <= Tolerance)
      {
        v.push_back(dist);
        distPrev = dist;
      }else{
        finish = true;
      }

      j++;
    }
    //Miro por encima
    distPrev = (float)croppedImage.at<float>(centerPixel.ycoor, column);
    j = centerPixel.ycoor - 1;
    finish = false;
    while(! finish && j >= 0)
    {
      dist = (float)croppedImage.at<float>(j, column);
      if(abs(dist - distPrev) <= Tolerance)
      {
        v.push_back(dist);
        distPrev = dist;
      }else{
        finish = true;
      }

      j--;
    }

  }

  void flood(cv::Mat croppedImage, darknet_ros_3d_msgs::BoundingBox3d &data)
  {
    std::vector<float> v; //distances vector
    float dist, distPrev;
    Type_Coord centerPixel;
    centerPixel.xcoor = int(croppedImage.size().width / 2);
    centerPixel.ycoor = int(croppedImage.size().height / 2);

    distPrev = (float)croppedImage.at<float>(centerPixel.ycoor, centerPixel.xcoor);
    v.push_back(distPrev);
    //Miro por la derecha
    int i = centerPixel.xcoor + 1;
    bool finish = false;
    while(! finish && i < croppedImage.size().width)
    {
      dist = (float)croppedImage.at<float>(centerPixel.ycoor, i);

      if(abs(dist - distPrev) <= Tolerance)
      {
        v.push_back(dist);
        distPrev = dist;
        //mirar todos los pixeles del eje y ;
        lookColumn(v, i, croppedImage);

      }else{
        finish = true;
      }

      i++;
    }
    //Miro por la izquierda
    distPrev = (float)croppedImage.at<float>(centerPixel.ycoor, centerPixel.xcoor);
    i = centerPixel.xcoor - 1;
    finish = false;
    while(! finish && i >= 0)
    {
      dist = (float)croppedImage.at<float>(centerPixel.ycoor, i);

      if(abs(dist - distPrev) <= Tolerance)
      {
        v.push_back(dist);
        distPrev = dist;
        //mirar todos los pixeles del eje y ;
        lookColumn(v, i, croppedImage);

      }else{
        finish = true;
      }

      i--;
    }
    data.depth.zmax = getMax(v) / 1000.0; //Distance is saved in meters
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

  float getDepth(darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg,
                              darknet_ros_3d_msgs::BoundingBox3d &data)
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
    nearestPixel = Darknet3d::getMinDist(croppedImage) / 1000.0; //Distance is saved in meters
    data.depth.zmin = nearestPixel;

    //Algorithm to disregard very distant pixels and calculate most far pixel
    Darknet3d::flood(croppedImage, data);
  }

  void publishMarker(std::vector<darknet_ros_3d_msgs::BoundingBox3d> v)
  {
    visualization_msgs::Marker marker; // el mensaje que se enviara por el topic

    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.7; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";


    marker.pose.position.x = (v[0].depth.zmin + v[0].depth.zmax) / 2.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    ROS_WARN("Publishing marker");
    _marker_pub.publish(marker);

    //Compongo el array de puntos [] points
    /*
    for(int i = 0; i < v.size(); i++)
    {
      marker.id = i;
      //este seria el centro del cubo
      //marker.pose.position.x = 0.2;
      //marker.pose.position.y = 0.2;
      //marker.pose.position.z = 3.5;

      marker.pose.position.x = (v[i].depth.zmin + v[i].depth.zmax) / 2.0;
      marker.pose.position.y = (v[i].xmin + v[i].xmax) / 2.0;
      marker.pose.position.z = (v[i].ymin + v[i].ymax) / 2.0;
      ROS_WARN("Publishing marker");
      _marker_pub.publish(marker);
    }
    */
  }

  void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    _cameraHeight = msg->height;
    _cameraWidth = msg->width;
    _cameraInfo_sub.shutdown();
  }

  void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if(isActive())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*msg, *pcrgb);

      if(_arrived && _originalBBoxes.size() > 0)
      {
        float XYmin, XYmax;
        for(int i = 0; i < _originalBBoxes.size(); i++)
        {
          ;
        }
      }
      pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
      tf::StampedTransform transform;
      /*
      for(int i = 0; i < _cameraWidth * _cameraHeight; i++)
      {
        //ROS_INFO("X = %f", pcrgb->at(i).x);
      }
      */
      /*
      for(it=pcrgb->begin(); it!=pcrgb->end(); ++it)
      {
        ROS_INFO("X = %f", it->x);
        ROS_INFO("Y = %f", it->y);
        ROS_INFO("Z = %f", it->z);
      }
      */
    }

  }

  void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    if(isActive())
    {
      _arrived = true;
      _originalBBoxes = msg->bounding_boxes;
    }
  }
  /*
  void depthCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    if(isActive())
    {
      std::vector<darknet_ros_3d_msgs::BoundingBox3d> v;
      darknet_ros_3d_msgs::BoundingBox3d data;

      if(_arrived && _originalBBoxes.size() > 0)
      {
        float dist;
        for(int i = 0; i < _originalBBoxes.size(); i++)
        {
          //Componer el mensaje
          data.Class = _originalBBoxes[i].Class;
          data.probability = _originalBBoxes[i].probability;
          data.xmin = _originalBBoxes[i].xmin;
          data.ymin = _originalBBoxes[i].ymin;
          data.xmax = _originalBBoxes[i].xmax;
          data.ymax = _originalBBoxes[i].ymax;

          Darknet3d::getDepth(_originalBBoxes[i], msg, data);
          //data.depth = dist / 1000.0; //save distance in meters
          //meterlo en el array
          v.push_back(data);
        }
        //Publicar el array
        darknet_ros_3d_msgs::BoundingBoxes3d msg_to_publish;
        msg_to_publish.bounding_boxes = v;
        _darknet3d_pub.publish(msg_to_publish);

        _arrived = false;

        //Publico el visual marker
        Darknet3d::publishMarker(v);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.3;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        _marker_pub.publish( marker );

      }
    }
  }
  */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darknet_3d");
  Darknet3d darknet3d;
  ros::Rate loop_rate(10); //10 Hz

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::spin();
  return 0;
}
