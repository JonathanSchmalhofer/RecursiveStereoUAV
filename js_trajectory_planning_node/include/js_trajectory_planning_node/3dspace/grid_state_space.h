#ifndef JS_TRAJECTORY_PLANNING_NODE_GRIDSTATESPACE_H
#define JS_TRAJECTORY_PLANNING_NODE_GRIDSTATESPACE_H

#include <Eigen/Dense>
#include "3dspace/obstacle_grid.h"
#include "3dspace/space_state_space.h"

namespace RRT
{

/// @brief A 3d space with continuous states and discretized obstacles.
/// @details The state space is broken up into a grid with the given discrete
/// height and widths and depths.
class GridStateSpace
    : public SpaceStateSpace
{
public:
    GridStateSpace(double width,
                   double height,
                   double depth,
                   int discretized_width,
                   int discretized_height,
                   int discretized_depth);

    /// @brief Returns a boolean indicating whether the given point is within
    /// bounds and obstacle-free.
    bool IsStateValid(const Eigen::Vector3d& pt) const;
    bool IsTransitionValid(const Eigen::Vector3d& from,
                           const Eigen::Vector3d& to) const;

    Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                         const Eigen::Vector3d& target,
                                         double min_step_size,
                                         double max_step_size) const;

    const ObstacleGrid& GetObstacleGrid() const;
    ObstacleGrid& GetObstacleGrid();

private:
    ObstacleGrid obstacle_grid_;
};

}  // namespace RRT

#endif // JS_TRAJECTORY_PLANNING_NODE_GRIDSTATESPACE_H
