#include <gtest/gtest.h>
#include <memory>
#include "robojackets_3d_trajectory_planning_node/3dspace/space_state_space.h"
#include <vector>

using namespace Eigen;
using namespace std;

namespace RRT
{

// SpaceStateSpace is an abstract class not implementing all virtual methods it
// inherits from StateSpace. To be able to instantiate it, all non-implemented
// virtual methods will be provided here as dummy-implementations.
class TestSpaceStateSpace
    : public SpaceStateSpace
{
public:
    TestSpaceStateSpace(double width,
                    double height,
                    double depth)
        : SpaceStateSpace(width, height, depth) {}
    ~TestSpaceStateSpace() {}

    // Dummy Implementation - will not be used/tested
    Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                         const Eigen::Vector3d& target,
                                         double step_size,
                                         double max_step_size) const
    {
        return Eigen::Vector3d(0,0,0);
    }

    // Implementation needed to call parent function
    Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                         const Eigen::Vector3d& target,
                                         double step_size) const
    {
        return SpaceStateSpace::GetIntermediateState(source, target, step_size);
    }

    // Dummy Implementation - will not be used/tested
    bool IsTransitionValid(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
    {
        return true;
    }

    double GetMaxStepSize() const { return max_step_size_; }
    double max_step_size_;
};

TEST(SpaceStateSpace, Instatiate)
{
    double width = 50.0f, height= 60.0f, depth = 70.0f;
    TestSpaceStateSpace space_state_space(width, height, depth);

    EXPECT_EQ(space_state_space.GetWidth(), width);
    EXPECT_EQ(space_state_space.GetHeight(), height);
    EXPECT_EQ(space_state_space.GetDepth(), depth);
}

TEST(SpaceStateSpace, CheckRandomPointsValid)
{
    double width = 50.0f, height= 60.0f, depth = 70.0f;
    TestSpaceStateSpace space_state_space(width, height, depth);

    for (int i = 0; i < 1000; i++)
    {
        Eigen::Vector3d random_point = space_state_space.GetRandomState();
        bool x_greater_equal_zero = random_point.x() >= 0;
        bool y_greater_equal_zero = random_point.y() >= 0;
        bool z_greater_equal_zero = random_point.z() >= 0;
        bool x_smaller_equal_width  = random_point.x() <= width;
        bool y_smaller_equal_height = random_point.y() <= height;
        bool z_smaller_equal_depth  = random_point.z() <= depth;
        EXPECT_EQ(x_greater_equal_zero, true);
        EXPECT_EQ(y_greater_equal_zero, true);
        EXPECT_EQ(z_greater_equal_zero, true);
        EXPECT_EQ(x_smaller_equal_width, true);
        EXPECT_EQ(y_smaller_equal_height, true);
        EXPECT_EQ(z_smaller_equal_depth, true);
        EXPECT_EQ(space_state_space.IsStateValid(random_point), true);
    }
}

TEST(SpaceStateSpace, CheckDistances)
{
    double width = 500.0f, height= 600.0f, depth = 700.0f;
    TestSpaceStateSpace space_state_space(width, height, depth);

    for (int i = 0; i < 1000; i++)
    {
        Eigen::Vector3d random_point_a = space_state_space.GetRandomState();
        Eigen::Vector3d random_point_b = space_state_space.GetRandomState();
        Eigen::Vector3d delta_a_b = random_point_a - random_point_b;
        double distance_a_b = delta_a_b.norm();
        double distance_state_space = space_state_space.GetDistance(random_point_a,random_point_b);
        double ratio = distance_a_b / distance_state_space;
        bool equal_within_numeric_bounds = (ratio < 1.01) && (ratio > 0.99);
        EXPECT_EQ(equal_within_numeric_bounds, true);
    }
}

TEST(SpaceStateSpace, CheckIntermediateStatesCorrect)
{
    double width = 500.0f, height= 600.0f, depth = 700.0f;
    TestSpaceStateSpace space_state_space(width, height, depth);

    for (int i = 0; i < 1000; i++)
    {
        Eigen::Vector3d random_point_a = space_state_space.GetRandomState();
        Eigen::Vector3d random_point_b = space_state_space.GetRandomState();
        Eigen::Vector3d delta_a_b = random_point_a - random_point_b;
        double distance_a_b = delta_a_b.norm();
        double random_step_size = drand48() * distance_a_b;
        space_state_space.max_step_size_ = random_step_size;

        // Get an intermediate state
        const Eigen::Vector3d intermediate_point = space_state_space.GetIntermediateState(random_point_a,
                                                                                    random_point_b,
                                                                                    space_state_space.GetMaxStepSize());

        // Check that the intermediate point's distance is smaller than the distance(a,b)
        bool distance_intermediate_to_a_smaller = space_state_space.GetDistance(random_point_a, intermediate_point) < distance_a_b;
        bool distance_intermediate_to_b_smaller = space_state_space.GetDistance(random_point_b, intermediate_point) < distance_a_b;
        EXPECT_EQ(distance_intermediate_to_a_smaller, true);
        EXPECT_EQ(distance_intermediate_to_b_smaller, true);
    }
}

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
