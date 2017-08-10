///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Collection of Unit Tests for the AirSimToRos Node
///

#include <gtest/gtest.h>
#include <ros/time.h>
#include "airsimros_airsim_to_ros/airsim_to_ros.h"

TEST(AirSimToRosTestSuite, DefaultStatus)
{
    // ARRANGE
    auto airsim_to_ros = airsimros::AirSimToRos();

    // ACT
    auto airsim_to_ros_status = airsim_to_ros.GetStatus();

    // ASSERT
    ASSERT_EQ(airsim_to_ros_status, 0);
}

TEST(AirSimToRosTestSuite, SetAndGetStatus)
{
    // ARRANGE
    auto airsim_to_ros = airsimros::AirSimToRos();

    // ACT
    airsim_to_ros.SetStatus(42);
    auto airsim_to_ros_status = airsim_to_ros.GetStatus();

    // ASSERT
    ASSERT_EQ(airsim_to_ros_status, 42);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
