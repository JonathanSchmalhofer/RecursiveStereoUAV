#include <gtest/gtest.h>
#include <memory>
#include "robojackets_3d_trajectory_planning_node/3dspace/3dspace.h"
#include "robojackets_3d_trajectory_planning_node/3dspace/grid_state_space.h"
#include "robojackets_3d_trajectory_planning_node/tree.h"
#include <vector>

using namespace Eigen;
using namespace std;

namespace RRT
{
TEST(Tree, Example3dSpace)
{
    shared_ptr<Tree<Vector3d>> tree =
        GetTreeFor3dSpace(make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50),
                       Vector3d(40, 40, 40),  // goal point
                       5);                    // step size

    // give it plenty of iterations so it's not likely to fail
    const int max_iterations = 10000;
    tree->SetMaxIterations(max_iterations);
    tree->SetMaxDistanceToGoal(5);

    tree->SetStartState(Vector3d(10, 10, 10));
    bool success = tree->Run();  // Run with the given starting point
    ASSERT_TRUE(success);
}

TEST(Tree, FailOnImpossibleRequest)
{
    shared_ptr<Tree<Vector3d>> tree = GetTreeFor3dSpace(
        make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50),
        Vector3d(60, 60, 60),  // goal point outside the bounds of the state space
        5);                    // step size

    // give it plenty of iterations so it's not likely to fail
    const int max_iterations = 2000;
    tree->SetMaxIterations(max_iterations);
    tree->SetMaxDistanceToGoal(5);

    tree->SetStartState(Vector3d(10, 10, 10));
    bool success = tree->Run();  // Run with the given starting point
    ASSERT_FALSE(success);  // the rrt search should fail because the goal isn't
                            // reachable
}

TEST(Tree, GetPath)
{
    Vector3d start = {10, 10, 10}, goal = {40, 40, 40};
    shared_ptr<Tree<Vector3d>> tree =
        GetTreeFor3dSpace(make_shared<GridStateSpace>(50, 50, 50, 50, 50, 50),
                       goal,  // goal point
                       5);    // step size

    // give it plenty of iterations so it's not likely to fail
    const int max_iterations = 10000;
    tree->SetMaxIterations(max_iterations);
    tree->SetMaxDistanceToGoal(5);

    tree->SetStartState(start);
    bool success = tree->Run();  // Run with the given starting point
    ASSERT_TRUE(success);

    // get path in reverse order (end -> root)
    vector<Vector3d> path;
    tree->GetPath(&path, tree->GetLastNode(), true);
    ASSERT_TRUE(path.size() > 1);
    EXPECT_EQ(start, path.back());

    // get path in regular order (root -> end)
    path.clear();
    tree->GetPath(&path, tree->GetLastNode(), false);
    ASSERT_TRUE(path.size() > 1);
    EXPECT_EQ(start, path.front());
}

TEST(Tree, AdaptiveScaling)
{
    // test adaptive stepsize control
    shared_ptr<RRT::Tree<Eigen::Vector3d>> tree =
        GetTreeFor3dSpace(make_shared<RRT::GridStateSpace>(800, 800, 800, 40, 40,40),
                       Eigen::Vector3d(790, 790, 790), // goal point
                       10);                     // step size

    // give it plenty of iterations so it's not likely to fail
    const int max_iterations = 10000;
    tree->SetMaxIterations(max_iterations);
    tree->SetMaxDistanceToGoal(5);
    tree->SetMaxStepSize(50);
    tree->SetAdaptiveScalingEnable(true);
    tree->SetGoalBias(0.2);

    tree->SetStartState(Eigen::Vector3d(10, 10, 10));
    bool success = tree->Run();  // Run with the given starting point
    ASSERT_TRUE(success);

    vector<Eigen::Vector3d> path;
    tree->GetPath(&path, tree->GetLastNode(), true);

    bool adaptive_scaling_enabled = tree->IsAdaptiveScalingEnable();
    ASSERT_TRUE(adaptive_scaling_enabled);

    // Check to see if the nodes in the tree have uniform stepsize or varied.
    // Stepsizes should vary
    bool varied = false;
    for (int i = 1; !varied && i < path.size() - 2; i++)
    {
        Vector3d path_one = path[i] - path[i - 1];
        Vector3d path_two = path[i] - path[i + 1];
        double n = path_one.norm() / path_two.norm();
        if (n < 0.99 || n > 1.01) varied = true;
    }
    EXPECT_EQ(varied, true);
}

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
