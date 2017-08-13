#include <gtest/gtest.h>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

TEST(AirSimToRosClassTestSuite, DefaultStatus)
{
    // ARRANGE
    auto airsim_to_ros = js_airsim_to_ros_library::AirSimToRosClass();

    // ACT
    auto airsim_to_ros_status = airsim_to_ros.GetStatus();

    // ASSERT
    ASSERT_EQ(airsim_to_ros_status, 0);
}

TEST(AirSimToRosClassTestSuite, SetAndGetStatus)
{
    // ARRANGE
    auto airsim_to_ros = js_airsim_to_ros_library::AirSimToRosClass();

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
