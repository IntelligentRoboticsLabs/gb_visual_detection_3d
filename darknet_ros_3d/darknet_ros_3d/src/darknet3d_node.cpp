#include <ros/ros.h>

#include "darknet_ros_3d/Darknet3D.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_3d");
  darknet_ros_3d::Darknet3D darknet3d;

  ros::Rate loop_rate(2);

  while (ros::ok())
  {
    darknet3d.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
