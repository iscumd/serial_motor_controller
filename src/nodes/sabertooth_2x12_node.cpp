#include <ros/ros.h>
#include "sabertooth_2x12.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sabertooth_2x12_node");

  serial_motor_controller::sabertooth_2x12 sabertooth;

  ros::Rate rt(1);

  while (!sabertooth.startup() && ros::ok())
  {
    rt.sleep();
    ROS_INFO("Attempting to connect to sabertooth!");
  }

  ros::spin();

  return 0;
}
