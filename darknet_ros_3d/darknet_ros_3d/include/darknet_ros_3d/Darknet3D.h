#ifndef DARKNET_RO_3D_DARKNET3D_H
#define DARKNET_RO_3D_DARKNET3D_H

#include <ros/ros.h>

#include <darknet_ros_3d_msgs/BoundingBoxes3d.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <vector>
#include <string>

namespace darknet_ros_3d
{

class Darknet3D
{
public:
	Darknet3D();

	virtual void update();

private:
	void initParams();
	void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
	void publish_markers(const darknet_ros_3d_msgs::BoundingBoxes3d& boxes);

	ros::NodeHandle nh_;
	ros::Subscriber yolo_sub_, pointCloud_sub_;
	ros::Publisher darknet3d_pub_, markers_pub_;
	tf::TransformListener tfListener_;

	std::vector<darknet_ros_msgs::BoundingBox> original_bboxes_;
	sensor_msgs::PointCloud2 point_cloud_;
	ros::Time last_detection_ts_;

	std::string input_bbx_topic_;
	std::string output_bbx3d_topic_;
	std::string pointcloud_topic_;
	std::string working_frame_;
  std::vector<std::string> interested_classes_;
  float mininum_detection_thereshold_;
};

};  // namespace darknet_ros_3d

#endif  // DARKNET_RO_3D_DARKNET3D_H
