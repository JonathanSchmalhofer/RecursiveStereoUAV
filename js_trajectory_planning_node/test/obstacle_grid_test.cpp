#include <gtest/gtest.h>
#include <memory>
#include "js_trajectory_planning_node/3dspace/obstacle_grid.h"
#include <vector>

using namespace Eigen;
using namespace std;

namespace RRT
{

TEST(ObstacleGrid, Instatiate)
{
    double width = 50.0f, height= 60.0f, depth = 70.0f;
    int discretized_width = 42, discretized_height= 63, discretized_depth = 74;
    ObstacleGrid obstacle_grid(width,
                               height,
                               depth,
                               discretized_width,
                               discretized_height,
                               discretized_depth);

    EXPECT_EQ(obstacle_grid.GetWidth(), width);
    EXPECT_EQ(obstacle_grid.GetHeight(), height);
    EXPECT_EQ(obstacle_grid.GetDepth(), depth);
    EXPECT_EQ(obstacle_grid.GetDiscretizedWidth(), discretized_width);
    EXPECT_EQ(obstacle_grid.GetDiscretizedHeight(), discretized_height);
    EXPECT_EQ(obstacle_grid.GetDiscretizedDepth(), discretized_depth);
}

TEST(ObstacleGrid, GetGridSquareForLocation)
{
    double width = 10.0f, height= 10.0f, depth = 10.0f;
    int discretized_width = 10, discretized_height= 10, discretized_depth = 10;
    ObstacleGrid obstacle_grid(width,
                               height,
                               depth,
                               discretized_width,
                               discretized_height,
                               discretized_depth);

    Eigen::Vector3d continues_point(0.4,0.5,2.4);
    Eigen::Vector3i discrete_point = obstacle_grid.GetGridSquareForLocation(continues_point);
    EXPECT_EQ(discrete_point.x(), 0);
    EXPECT_EQ(discrete_point.y(), 1);
    EXPECT_EQ(discrete_point.z(), 2);
}
/*
TEST(ObstacleGrid, Initialize)
{
}

TEST(ObstacleGrid, OcTree)
{
}

TEST(ObstacleGrid, NearestObstacle)
{
}

TEST(ObstacleGrid, Clear)
{
}

TEST(ObstacleGrid, ObstaclePositions)
{
}

TEST(ObstacleGrid, InsertedRays)
{
}

TEST(ObstacleGrid, InsertedMeasurements)
{
}

TEST(ObstacleGrid, CheckCollisionFreeLines)
{
}*/

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
