/**
* @file   test_sample_cpp_node.cpp
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
* @brief Test for sample C++ node.
*
*/

#include <gtest/gtest.h>
#include <string>
#include "ros/ros.h"

TEST(SampleCppTestSuite, checkParameterValue)
{
  const ros::M_string remapping_args;
  ros::init(remapping_args, "test_sample_cpp_node");
  const ros::NodeHandle nh;

  ros::Rate rate(2);
  ros::spinOnce();
  rate.sleep();

  std::string param_value;
  const bool result = nh.getParam("my_cpp_param", param_value);
  const std::string expected_value("my_cpp_param_test");

  ASSERT_TRUE(result);
  ASSERT_EQ(expected_value, param_value);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
