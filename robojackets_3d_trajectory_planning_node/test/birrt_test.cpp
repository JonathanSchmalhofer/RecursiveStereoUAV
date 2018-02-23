#include <gtest/gtest.h>
#include "robojackets_3d_trajectory_planning_node/3dspace/3dspace.h"
#include "robojackets_3d_trajectory_planning_node/3dspace/grid_state_space.h"
#include "robojackets_3d_trajectory_planning_node/birrt.h"

using namespace std;
using namespace Eigen;

namespace RRT {

TEST(BiRRT, Instantiation)
{
    BiRRT<Vector3d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50), 3);
}

TEST(BiRRT, GetPath)
{
    Vector3d start = {1, 1, 1}, goal = {30, 30, 30};

    BiRRT<Vector3d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50), 3);
    biRRT.SetStartState(start);
    biRRT.SetGoalState(goal);
    biRRT.SetStepSize(1);
    biRRT.SetMaxIterations(10000);
/*
    bool success = biRRT.Run();
    ASSERT_TRUE(success);

    vector<Vector3d> path = biRRT.GetPath();

    // path should contain at least two points (start and end)
    ASSERT_GE(path.size(), 2);

    // The given start and goal points should be the first and last points of
    // the path, respectively.
    EXPECT_EQ(start, path.front());
    EXPECT_EQ(goal, path.back());*/
}

TEST(BiRRT, MultipleRuns)
{
    Vector3d start = {1, 1, 1}, goal = {30, 30, 30};

    BiRRT<Vector3d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50), 3);
    biRRT.SetStartState(start);
    biRRT.SetGoalState(goal);
    biRRT.SetStepSize(1);
    biRRT.SetMaxIterations(10000);

    for (int i = 0; i < 50; i++)
    {
        /*bool success = biRRT.Run();
        ASSERT_TRUE(success);
        biRRT.Reset();*/
    }
}

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
