#include <ros/ros.h>

#include "darknet_ros_3d/Darknet3D.h"
#include <bica/Component.h>

class Darknet3DBica: public darknet_ros_3d::Darknet3D, bica::Component
{
public:
  void update()
  {
    if (isActive())
      darknet_ros_3d::Darknet3D::update();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_3d");
  Darknet3DBica darknet3d;

  ros::Rate loop_rate(darknet3d.execution_frequency_);

  while (ros::ok())
  {
    darknet3d.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
