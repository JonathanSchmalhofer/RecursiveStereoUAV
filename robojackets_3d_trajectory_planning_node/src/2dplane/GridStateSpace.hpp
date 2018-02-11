#pragma once

#include <Eigen/Dense>
#include <2dplane/ObstacleGrid.hpp>
#include <2dplane/PlaneStateSpace.hpp>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and discretized obstacles.
 * @details The state space is broken up into a grid with the given discrete
 * height and widths.
 */
class GridStateSpace : public PlaneStateSpace {
public:
    GridStateSpace(double width, double height, double depth, int discretizedWidth,
                   int discretizedHeight, int discretizedDepth);

    /**
     * Returns a boolean indicating whether the given point is within bounds and
     * obstacle-free.
     */
    bool stateValid(const Eigen::Vector3d& pt) const;
    bool transitionValid(const Eigen::Vector3d& from,
                         const Eigen::Vector3d& to) const;

    Eigen::Vector3d intermediateState(const Eigen::Vector3d& source,
                                      const Eigen::Vector3d& target,
                                      double minStepSize,
                                      double maxStepSize) const;

    const ObstacleGrid& obstacleGrid() const;
    ObstacleGrid& obstacleGrid();

private:
    ObstacleGrid _obstacleGrid;
};

}  // namespace RRT
