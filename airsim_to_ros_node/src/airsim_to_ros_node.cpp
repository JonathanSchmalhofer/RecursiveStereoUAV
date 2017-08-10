
#include <string>
#include "ros/ros.h"
#include "airsimros_airsim_to_ros/airsim_to_ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "airsim_to_ros_node");
  ros::NodeHandle nh;

  airsimros::AirSimToRos airsim_to_ros;

  ros::Rate r(5);

  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
