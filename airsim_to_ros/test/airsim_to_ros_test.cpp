///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Collection of Unit Tests for the AirSimToRos Node
///

#include <gtest/gtest.h>
#include <ros/time.h>

#include "airsimros_airsim_to_ros/airsim_to_ros.h"

namespace airsimros
{
TEST(AirSimToRosTest, DefaultStatus)
{
    ros::Time::init();

    // ARRANGE
    auto airsim_to_ros = AirSimToRos();

    // ACT
    auto airsim_to_ros_status = airsim_to_ros.GetStatus();

    // ASSERT
	EXPECT_EQ(airsim_to_ros_status, 0);
}

/*
TEST(AirSimToRosTest, SetAndGetStatus)
{
    ros::Time::init();

    // ARRANGE
    auto airsim_to_ros = AirSimToRos();

    // ACT
	airsim_to_ros.SetStatus(42);
    auto airsim_to_ros_status = airsim_to_ros.GetStatus();

    // ASSERT
	EXPECT_EQ(airsim_to_ros_status, 42);
}
*/
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
