#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H

#include <Eigen/Dense>

namespace RRT
{

/// @brief Handles a grid of obstacles laid over a continuous 3d space.
/// 
/// @details The state space is broken up into a grid with the given discrete
/// height and widths and depths.
class ObstacleGrid
{
public:
    ObstacleGrid(double width,
                 double height,
                 double depth,
                 int GetDiscretizedWidth,
                 int GetDiscretizedHeight,
                 int GetDiscretizedDepth);
    ~ObstacleGrid();

    Eigen::Vector3i GetGridSquareForLocation(const Eigen::Vector3d& loc) const;


    /// Finds the distance from state to its neareset obstacle. Only searches up to
    /// max_distance around state so as to not waste time checking far away and irrelevant
    /// obstacles.
    /// 
    /// @param state The location to search with respect to for the nearest
    /// obstacle dist
    /// @param max_distance The maximum vertical and horizontal distance from state to
    /// search for obstacles
    double GetDistanceToNearestObstacle(const Eigen::Vector3d& state,
                                        double max_distance) const;
    void Clear();
    bool& IsObstacleAt(int x, int y, int z);
    bool IsObstacleAt(int x, int y, int z) const;
    bool& IsObstacleAt(const Eigen::Vector3i& grid_location);
    bool IsObstacleAt(const Eigen::Vector3i& grid_location) const;

    int GetDiscretizedWidth() const;
    int GetDiscretizedHeight() const;
    int GetDiscretizedDepth() const;
    double GetWidth() const;
    double GetHeight() const;
    double GetDepth() const;

private:
    int discretized_width_, discretized_height_, discretized_depth_;
    double width_, height_, depth_;

    /// @brief 3d array of obstacles
    bool* obstacles_;
};

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H
