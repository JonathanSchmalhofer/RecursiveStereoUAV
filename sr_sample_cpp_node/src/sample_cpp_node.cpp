/**
* @file   sample_cpp_node.cpp
* @author Andriy Petlovanyy <software@shadowrobot.com>
*
* Copyright 2015 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* @brief Sample C++ Node.
*
*/

#include <string>
#include "ros/ros.h"
#include "sr_sample_cpp_library/sample_cpp_class.h"

using sr_sample_cpp_library::SampleCppClass;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_cpp_node");
  ros::NodeHandle nh;

  SampleCppClass sample_object;

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
