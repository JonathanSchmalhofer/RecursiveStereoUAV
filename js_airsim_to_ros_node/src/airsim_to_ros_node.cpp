#include <string>
#include "ros/ros.h"
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "airsim_to_ros_node");
  ros::NodeHandle nh;

  js_airsim_to_ros_library::AirSimToRosClass sample_object;

  std::string param_name("my_cpp_param");
  std::string param_value = sample_object.getParameterValueFromName(param_name);

  nh.setParam(param_name, param_value);

  ros::Rate r(5);

  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
