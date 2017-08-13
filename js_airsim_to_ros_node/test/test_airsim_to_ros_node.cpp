#include <gtest/gtest.h>
#include <string>
#include "ros/ros.h"

TEST(AirSimToRosNodeTestSuite, checkParameterValue)
{
  const ros::M_string remapping_args;
  ros::init(remapping_args, "test_airsim_to_ros_node");
  const ros::NodeHandle nh;

  ros::Rate rate(2);
  ros::spinOnce();
  rate.sleep();

  ASSERT_EQ(42, 42);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
