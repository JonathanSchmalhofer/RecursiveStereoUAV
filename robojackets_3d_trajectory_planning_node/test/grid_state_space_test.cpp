#include <gtest/gtest.h>
#include <memory>
#include "robojackets_3d_trajectory_planning_node/3dspace/grid_state_space.h"
#include <vector>

using namespace Eigen;
using namespace std;

namespace RRT
{

TEST(GridStateSpace, Instatiate)
{
    GridStateSpace grid_state_space(50, 50, 50, 50, 50, 50);
}

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
