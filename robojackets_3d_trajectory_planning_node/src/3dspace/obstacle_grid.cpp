#include "obstacle_grid.h"
#include <stdlib.h>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace RRT
{

ObstacleGrid::ObstacleGrid(double width,
                           double height,
                           double depth,
                           int discretized_width,
                           int discretized_height,
                           int discretized_depth)
    : octree_obstacles_(NULL)
{
    width_ = width;
    height_ = height;
    depth_ = depth;
    discretized_width_ = discretized_width;
    discretized_height_ = discretized_height;
    discretized_depth_ = discretized_depth;

    obstacles_ =
        (bool*)malloc(sizeof(bool) * discretized_width * discretized_height * discretized_depth);
    Initialize();
    Clear();
}

ObstacleGrid::~ObstacleGrid()
{
    free(obstacles_);
    if(octree_obstacles_ != NULL)
    {
        delete octree_obstacles_;
    }
}

void ObstacleGrid::Initialize()
{
    if(octree_obstacles_ != NULL)
    {
        delete octree_obstacles_;
    }
    octree_obstacles_ = new octomap::OcTree(kresolution_octomap);

    // add test wall
    ROS_INFO("Make Octomap great again");
    double x_wall, y_wall, z_wall;
    y_wall = 40;
    for(x_wall = -0.3*kminimum_uniform_extent_x; x_wall <= 0.3*kminimum_uniform_extent_x; x_wall+=0.5*kresolution_octomap)
    {
        for(z_wall = -0.3*kminimum_uniform_extent_z; z_wall <= 0.3*kminimum_uniform_extent_z; z_wall+=0.5* kresolution_octomap)
        {
            octomap::point3d start(x_wall, y_wall-5, z_wall);
            octomap::point3d end(x_wall, y_wall, z_wall);

            //ROS_INFO("%f, %f, %f", x_wall, y_wall, z_wall);

            octree_obstacles_->insertRay(start, end);
        }
    }
    ROS_INFO("Trump was here");
}

octomap::OcTree* ObstacleGrid::GetOctTreeObstacles()
{
    return octree_obstacles_;
}

Vector3i ObstacleGrid::GetGridSquareForLocation(const Vector3d& loc) const
{
    return Vector3i(loc.x() / GetWidth() * GetDiscretizedWidth(),
                    loc.y() / GetHeight() * GetDiscretizedHeight(),
                    loc.z() / GetDepth() * GetDiscretizedDepth());
}

double ObstacleGrid::GetDistanceToNearestObstacle(const Vector3d& state,
                                                  double max_distance) const
{
    // x and y are the indices of the cell that state is located in
    double x = (state.x() / (width_ / discretized_width_));
    double y = (state.y() / (height_ / discretized_height_));
    double z = (state.z() / (depth_ / discretized_depth_));
    int search_radius_x = max_distance * discretized_width_ / width_;
    int search_radius_y = max_distance * discretized_height_ / height_;
    int search_radius_z = max_distance * discretized_depth_ / depth_;
    // here we loop through the cells around (x,y,z) to find the minimum distance
    // of the point to the nearest obstacle
    for (int i = x - search_radius_x; i <= x + search_radius_x; i++)
    {
        for (int j = y - search_radius_y; j <= y + search_radius_y; j++)
        {
            for (int k = z - search_radius_z; k <= z + search_radius_z; k++)
            {
                bool obs = IsObstacleAt(i, j, k);
                if (obs)
                {
                    double x_distance = (x - i) * width_ / discretized_width_;
                    double y_distance = (y - j) * height_ / discretized_height_;
                    double z_distance = (z - k) * depth_ / discretized_depth_;
                    double dist = sqrtf(powf(x_distance, 2) + powf(y_distance, 2) + powf(z_distance, 2));
                    if (dist < max_distance)
                    {
                        max_distance = dist;
                    }
                }
            }
        }
    }

    // the boundaries of the grid count as obstacles
    max_distance = std::min(max_distance, state.x());             // left boundary
    max_distance = std::min(max_distance, GetWidth() - state.x());   // right boundary
    max_distance = std::min(max_distance, state.y());             // top boundary
    max_distance = std::min(max_distance, GetHeight() - state.y());  // bottom boundary
    max_distance = std::min(max_distance, state.z());             // front boundary
    max_distance = std::min(max_distance, GetDepth() - state.z());   // back boundary

    return max_distance;
}

void ObstacleGrid::Clear()
{
    for (int x = 0; x < GetDiscretizedWidth(); x++)
    {
        for (int y = 0; y < GetDiscretizedHeight(); y++)
        {
            for (int z = 0; z < GetDiscretizedDepth(); z++)
            {
                IsObstacleAt(x, y, z) = false;
            }
        }
    }
}

bool& ObstacleGrid::IsObstacleAt(int x, int y, int z)
{
    return obstacles_[x + discretized_width_ * y + discretized_depth_ * z];
}

bool ObstacleGrid::IsObstacleAt(int x, int y, int z) const
{
    return obstacles_[x + discretized_width_ * y + discretized_depth_ * z];
}

bool& ObstacleGrid::IsObstacleAt(const Vector3i& grid_location)
{
    return IsObstacleAt(grid_location.x(), grid_location.y(), grid_location.z());
}

bool ObstacleGrid::IsObstacleAt(const Vector3i& grid_location) const
{
    return IsObstacleAt(grid_location.x(), grid_location.y(), grid_location.z());
}

int ObstacleGrid::GetDiscretizedWidth() const { return discretized_width_; }

int ObstacleGrid::GetDiscretizedHeight() const { return discretized_height_; }

int ObstacleGrid::GetDiscretizedDepth() const { return discretized_depth_; }

double ObstacleGrid::GetWidth() const { return width_; }

double ObstacleGrid::GetHeight() const { return height_; }

double ObstacleGrid::GetDepth() const { return depth_; }

}  // namespace RRT
