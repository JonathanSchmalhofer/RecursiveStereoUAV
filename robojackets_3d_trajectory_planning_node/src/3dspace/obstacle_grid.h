#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

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

    /// @brief Initialize internally
    void Initialize();

    /// @brief Returns OcTree-obstacles
    octomap::OcTree* GetOctTreeObstacles();

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
    bool IsObstacleAt(int x, int y, int z) const;
    bool IsObstacleAt(const Eigen::Vector3d& grid_location) const;

    int GetDiscretizedWidth() const;
    int GetDiscretizedHeight() const;
    int GetDiscretizedDepth() const;
    double GetWidth() const;
    double GetHeight() const;
    double GetDepth() const;

    bool InsertRayOccupiedAtEnd(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    bool InsertOccupiedMeasurement(const Eigen::Vector3d& position);
    bool CheckIfCollisionFreeLineBetween(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

private:
    int discretized_width_, discretized_height_, discretized_depth_;
    double width_, height_, depth_;

    /// @brief Obstacles represented in an OcTree
    octomap::OcTree* octree_obstacles_;

    /// @brief Resolution of octomap
    const double kresolution_octomap = 1;

    /// @brief Minimum extent in x-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_x = 100;

    /// @brief Minimum extent in y-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_y = 100;

    /// @brief Minimum extent in z-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_z = 20;

};

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_OBSTACLE_GRID_H
