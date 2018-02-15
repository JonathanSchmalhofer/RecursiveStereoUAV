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

    Initialize();
    Clear();
}

ObstacleGrid::~ObstacleGrid()
{
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
/*
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
    ROS_INFO("Trump was here");*/
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
    /*
    double x,y,z;
    octree_obstacles_->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    octree_obstacles_->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);max_distance
    
    bool unknown_as_occupied = false;
    unknownAsOccupied = false;
    float max_clamped_distance = 1.0;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map
    DynamicEDTOctomap distmap(max_clamped_distance, octree_obstacles_, min, max, unknown_as_occupied);

    //This computes the distance map
    distmap.update(); 

    //This is how you can query the map
    octomap::point3d p(state.x(), state.y(), state.z());

    octomap::point3d closestObst;
    double distance;

    distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

    if(distance < distmap.getMaxDist() && distance < max_distance)
        return distance;
    else*/
        return max_distance;
}

void ObstacleGrid::Clear()
{
    octree_obstacles_->clear();
}

bool ObstacleGrid::IsObstacleAt(int x, int y, int z) const
{
    if(nullptr == octree_obstacles_) { return false; };

    octomap::OcTreeNode* node = octree_obstacles_->search(octomap::point3d(x,y,z));

    if(nullptr == node) { return false; };

    return octree_obstacles_->isNodeOccupied(node);
}

bool ObstacleGrid::IsObstacleAt(const Vector3d& grid_location) const
{
    return IsObstacleAt(grid_location.x(),grid_location.y(),grid_location.z());
}

int ObstacleGrid::GetDiscretizedWidth() const { return discretized_width_; }

int ObstacleGrid::GetDiscretizedHeight() const { return discretized_height_; }

int ObstacleGrid::GetDiscretizedDepth() const { return discretized_depth_; }

double ObstacleGrid::GetWidth() const { return width_; }

double ObstacleGrid::GetHeight() const { return height_; }

double ObstacleGrid::GetDepth() const { return depth_; }

bool ObstacleGrid::InsertRayOccupiedAtEnd(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
    if(octree_obstacles_)
    {
        octomap::point3d start(from.x(), from.y(), from.z());
        octomap::point3d end(to.x(), to.y(), to.z());
        return octree_obstacles_->insertRay(start, end);
    }
    else
    {
        return false;
    }
}

bool ObstacleGrid::InsertOccupiedMeasurement(const Eigen::Vector3d& position)
{
    if(octree_obstacles_)
    {
        octomap::point3d endpoint(position.x(), position.y(), position.z());
        octree_obstacles_->updateNode(endpoint, true); // integrate 'occupied' measurement
        return true;
    }
    else
    {
        return false;
    }
}

bool ObstacleGrid::CheckIfCollisionFreeLineBetween(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
{
    if(octree_obstacles_)
    {
        octomap::point3d start(from.x(), from.y(), from.z());
        octomap::point3d direction(to.x() - from.x(),
                                   to.y() - from.y(),
                                   to.z() - from.z());
        octomap::point3d cell_hit_by_ray;

        bool ignore_unknown_cells = true;
        double max_range = sqrtf(powf(to.x() - from.x(), 2) + powf(to.y() - from.y(), 2) + powf(to.z() - from.z(), 2)); // EuclidianDistance3d(x_a, x_b);
        bool collision_occured = octree_obstacles_->castRay(start, direction, cell_hit_by_ray, ignore_unknown_cells, max_range);
        return (!collision_occured);
    }
    else
    {
        return true;
    }
}

}  // namespace RRT
