

#include <gtest/gtest.h>
#include <string>
#include "ros/ros.h"

TEST(AirSimToRosNodeTestSuite, dummyTest)
{
  ASSERT_EQ(1, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
