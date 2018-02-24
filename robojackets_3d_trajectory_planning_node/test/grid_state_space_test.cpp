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
    double width = 50.0f, height= 60.0f, depth = 70.0f;
    GridStateSpace grid_state_space(width, height, depth, 50, 50, 50);

    EXPECT_EQ(grid_state_space.GetWidth(), width);
    EXPECT_EQ(grid_state_space.GetHeight(), height);
    EXPECT_EQ(grid_state_space.GetDepth(), depth);
}

TEST(GridStateSpace, CheckValidStatesInEmptySpace)
{
    double width = 50.0f, height= 60.0f, depth = 70.0f;
    GridStateSpace grid_state_space(width, height, depth, 50, 50, 50);

    // Zero Point inside
    bool zero_point_valid = grid_state_space.IsStateValid(Vector3d(0,0,0));
    EXPECT_EQ(zero_point_valid, true);
    // Corners outside
    bool corner_valid = grid_state_space.IsStateValid(Vector3d(width,0,0));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(0,height,0));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(0,0,depth));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(width,height,0));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(width,0,depth));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(0,height,depth));
    EXPECT_EQ(corner_valid, false);
    corner_valid = grid_state_space.IsStateValid(Vector3d(width,height,depth));
    EXPECT_EQ(corner_valid, false);
}

TEST(GridStateSpace, CheckValidStatesWithObstacles)
{
    double width = 500.0f, height= 500.0f, depth = 500.0f;
    GridStateSpace grid_state_space(width, height, depth, width, height, depth);

    //////////////////////////////////////////////////////////////////
    // Check Obstacle Space Boundaries
    EXPECT_EQ(grid_state_space.GetObstacleGrid().GetWidth(), width);
    EXPECT_EQ(grid_state_space.GetObstacleGrid().GetHeight(), height);
    EXPECT_EQ(grid_state_space.GetObstacleGrid().GetDepth(), depth);

    //////////////////////////////////////////////////////////////////
    // Enter random obstacles and check whether they are all returned as invalid states
    for (int i = 0; i < 500; i++)
    {
        // Random points in Grid Space with scale factor [0.0;0.99]
        double x = 0.99 * drand48(), y = 0.99 * drand48(), z = 0.99 * drand48();
        Vector3d new_point( x * width, y * height, z * depth );

        // Insert Obstacles
        EXPECT_EQ(grid_state_space.GetObstacleGrid().InsertOccupiedMeasurement(new_point), true);
        // State at obstacle should be invalid
        bool new_point_valid = grid_state_space.IsStateValid(new_point);
        EXPECT_EQ(new_point_valid, false);
    }

    //////////////////////////////////////////////////////////////////
    // Check if obstacles get cleared
    Vector3d middle_point( 0.5 * width, 0.5 * height, 0.5 * depth );

    // Insert Obstacles
    EXPECT_EQ(grid_state_space.GetObstacleGrid().InsertOccupiedMeasurement(middle_point), true);

    // State at obstacle should be invalid...
    bool middle_point_valid = grid_state_space.IsStateValid(middle_point);
    EXPECT_EQ(middle_point_valid, false);

    // ...but valid after clearing the obstacle space
    grid_state_space.GetObstacleGrid().Clear();
    middle_point_valid = grid_state_space.IsStateValid(middle_point);
    EXPECT_EQ(middle_point_valid, true);
}

TEST(GridStateSpace, CheckValidTransitionsWithObstacles)
{
    double width = 500.0f, height= 500.0f, depth = 500.0f;
    GridStateSpace grid_state_space(width, height, depth, width, height, depth);

    // Errect a "wall" parallel to the x-z-plane, parting the space along the
    // y-axis into two separate sub-spaces
    double x_wall, y_wall, z_wall, step_width;
    y_wall = 0.5* height;
    step_width = 0.25;
    for(x_wall = 0; x_wall <= width; x_wall += step_width)
    {
        for(z_wall = 0; z_wall <= depth; z_wall += step_width)
        {
            Vector3d wall_point(x_wall, y_wall, z_wall);
            EXPECT_EQ(grid_state_space.GetObstacleGrid().InsertOccupiedMeasurement(wall_point), true);
        }
    }

    // Now take random points from each of the two subspaces and check whether a straight
    // line between them hits the wall and thus makes any transition from Subspace A to
    // Subspace B invalid
    for (int i = 0; i < 500; i++)
    {
        // Random points in Grid Space
        //     Subspace A is scaled for [x,y,z] within [ [0,0,0],[0.99,0.40,0.99] ]
        //     Subspace B is scaled for [x,y,z] within [ [0,0.6,0],[0.99,0.99,0.99] ]
        double x_a = drand48() * 0.99;
        double y_a = drand48() * 0.40;
        double z_a = drand48() * 0.99;
        double x_b = drand48() * 0.99;
        double y_b = drand48() * 0.39 + 0.60;
        double z_b = drand48() * 0.99;
        Vector3d point_subspace_a( x_a * width, y_a * height, z_a * depth );
        Vector3d point_subspace_b( x_b * width, y_b * height, z_b * depth );

        // Transition from A to B and vice versa should not be possible, due to the "wall"
        bool transition_a_to_b = grid_state_space.IsTransitionValid(point_subspace_a, point_subspace_b);
        bool transition_b_to_a = grid_state_space.IsTransitionValid(point_subspace_b, point_subspace_a);
        EXPECT_EQ(transition_a_to_b, false);
        EXPECT_EQ(transition_b_to_a, false);
    }

    // Repeat after clearing the "wall", all transitions now should be valid
    grid_state_space.GetObstacleGrid().Clear();
    for (int i = 0; i < 500; i++)
    {
        // Random points in Grid Space
        //     Subspace A is scaled for [x,y,z] within [ [0,0,0],[0.99,0.40,0.99] ]
        //     Subspace B is scaled for [x,y,z] within [ [0,0.6,0],[0.99,0.99,0.99] ]
        double x_a = drand48() * 0.99;
        double y_a = drand48() * 0.40;
        double z_a = drand48() * 0.99;
        double x_b = drand48() * 0.99;
        double y_b = drand48() * 0.39 + 0.60;
        double z_b = drand48() * 0.99;
        Vector3d point_subspace_a( x_a * width, y_a * height, z_a * depth );
        Vector3d point_subspace_b( x_b * width, y_b * height, z_b * depth );

        // Transition from A to B and vice versa should not be possible, due to the "wall"
        bool transition_a_to_b = grid_state_space.IsTransitionValid(point_subspace_a, point_subspace_b);
        bool transition_b_to_a = grid_state_space.IsTransitionValid(point_subspace_b, point_subspace_a);
        EXPECT_EQ(transition_a_to_b, true);
        EXPECT_EQ(transition_b_to_a, true);
    }
}

TEST(GridStateSpace, CheckIntermediateStates)
{
    double width = 500.0f, height= 500.0f, depth = 500.0f;
    GridStateSpace grid_state_space(width, height, depth, width, height, depth);

    // Take random points from withing the (almost) entire grid state space
    // and check whether the intermediate state is calculated correctly
    for (int i = 0; i < 100; i++)
    {
        // Random points in Grid Space scaled for [x,y,z] within [ [0,0,0],[0.99,0.99,0.99] ]
        double x_a = drand48() * 0.99;
        double y_a = drand48() * 0.99;
        double z_a = drand48() * 0.99;
        double x_b = drand48() * 0.99;
        double y_b = drand48() * 0.99;
        double z_b = drand48() * 0.99;
        Vector3d point_a( x_a * width, y_a * height, z_a * depth );
        Vector3d point_b( x_b * width, y_b * height, z_b * depth );

        ////////////////////////////////////////////////////////////////////////
        // Calculate intermediate states into both directions for...

        // min_step_size = max_step_size = 0.5
        {
            Vector3d delta_a_b = point_a - point_b;
            double distance_a_b = delta_a_b.norm();

            // A --> B
            Vector3d between_a_to_b = grid_state_space.GetIntermediateState(point_a,
                                                                            point_b,
                                                                           0.5 * distance_a_b,
                                                                           0.5 * distance_a_b);
            Vector3d delta_a_to_new = between_a_to_b - point_a;
            double distance_a_to_new = delta_a_to_new.norm();
            bool step_approx_half_distance_from_a_to_new = (distance_a_to_new <= 0.51 * distance_a_b) && (distance_a_to_new >= 0.49 * distance_a_b);
            EXPECT_EQ(step_approx_half_distance_from_a_to_new, true);
        }

        // ... 500 variations with [min_step_size;max_step_size] = [[0.1;0.3],[0.4;0.9] * distance(a,b)
        // HINT: GetIntermediateState(...) with 4 arguments for GridStateSpace always has adaptive
        //       scaling enabled.
        for (int j = 0; j < 500; j++)
        {
            double scale_factor_min = drand48() * 0.2 + 0.1;
            double scale_factor_max = drand48() * 0.5 + 0.4;
            Vector3d delta_a_b = point_a - point_b;
            double distance_a_b = delta_a_b.norm();

            // A --> B
            Vector3d between_a_to_b = grid_state_space.GetIntermediateState(point_a,
                                                                            point_b,
                                                                           scale_factor_min * distance_a_b,
                                                                           scale_factor_max * distance_a_b);
            Vector3d delta_a_to_new = between_a_to_b - point_a;
            double distance_a_to_new = delta_a_to_new.norm();
            bool step_farther_than_min_step_size = distance_a_to_new >= 0.49 * scale_factor_min * distance_a_b;
            bool step_less_than_max_step_size    = distance_a_to_new <= 1.01 * scale_factor_max * distance_a_b;
            EXPECT_EQ(step_farther_than_min_step_size, true);
            EXPECT_EQ(step_less_than_max_step_size, true);

            // B --> A
            Vector3d between_b_to_a = grid_state_space.GetIntermediateState(point_b,
                                                                            point_a,
                                                                           scale_factor_min * distance_a_b,
                                                                           scale_factor_max * distance_a_b);
            Vector3d delta_b_to_new = between_b_to_a - point_b;
            double distance_b_to_new = delta_b_to_new.norm();
            step_farther_than_min_step_size = distance_b_to_new >= 0.49 * scale_factor_min * distance_a_b;
            step_less_than_max_step_size    = distance_b_to_new <= 1.01 * scale_factor_max * distance_a_b;
            EXPECT_EQ(step_farther_than_min_step_size, true);
            EXPECT_EQ(step_less_than_max_step_size, true);
        }
    }
}

}  // namespace RRT

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
