#include "robojackets_3d_trajectory_planning_node/3dspace/obstacle_grid.h"
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

    if(distance_map_ != NULL)
    {
        delete distance_map_;
    }
}

void ObstacleGrid::Initialize()
{
    if(octree_obstacles_ != NULL)
    {
        delete octree_obstacles_;
    }
    octree_obstacles_ = new octomap::OcTree(kresolution_octomap);

    if(distance_map_ != NULL)
    {
        delete distance_map_;
    }
    InitDistanceMap();
}

bool ObstacleGrid::InitDistanceMap()
{
    if (octree_obstacles_ != nullptr &&
        distance_map_     == nullptr)
    {
        double x_min,y_min,z_min,x_max,y_max,z_max;
        octree_obstacles_->getMetricMin(x_min,y_min,z_min);
        octree_obstacles_->getMetricMax(x_max,y_max,z_max);

        double distance_max_min = sqrtf(powf(x_max-x_min, 2) + powf(y_max-y_min, 2) + powf(z_max-z_min, 2));

        //ROS_INFO("x_min = %d, y_min = %d, z_min = %d", x_min, y_min, z_min);
        //ROS_INFO("x_max = %d, y_max = %d, z_max = %d", x_max, y_max, z_max);
        if (distance_max_min == 0)
        {
            x_min = 0; y_min = 0; y_max = 0;
            x_max = 0.1*width_;
            y_max = 0.1*height_;
            z_max = 0.1*depth_;
            //ROS_INFO("Max Bounds");
        }
        octomap::point3d min(x_min,y_min,z_min);
        octomap::point3d max(x_max,y_max,z_max);

        bool unknown_as_occupied = false;
        float max_clamped_distance = 20; // sqrtf(powf(width_, 2) + powf(height_, 2) + powf(depth_, 2));
        //- the first argument ist the max distance at which distance computations are clamped
        //- the second argument is the octomap
        //- arguments 3 and 4 can be used to restrict the distance map to a subarea
        //- argument 5 defines whether unknown space is treated as occupied or free
        //The constructor copies data but does not yet compute the distance map
        distance_map_ = new DynamicEDTOctomap (max_clamped_distance, octree_obstacles_, min, max, unknown_as_occupied);

        //This computes the distance map
        distance_map_->update();

        return true;
    }
    return false;
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
    if (distance_map_ != NULL)
    {
        //This computes the distance map
        distance_map_->update(); 

        //This is how you can query the map
        octomap::point3d p(state.x(), state.y(), state.z());

        octomap::point3d closest_obstacle;
        float distance;

        distance_map_->getDistanceAndClosestObstacle(p, distance, closest_obstacle);

        /*if (distance != distance_map_->distanceValue_Error)
        {
            ROS_INFO("No Distance Value could be retreived");
        }*/

        if(distance < distance_map_->getMaxDist() && distance < max_distance && distance != distance_map_->distanceValue_Error)
        {
            //ROS_INFO("Found a Obstacle close");
            return static_cast<double>(distance);
        }
        else
        {
            return max_distance;
        }
    }
    else
    {
        //ROS_INFO("No distance map available");
        return max_distance;
    }
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
        bool return_value = octree_obstacles_->insertRay(start, end);
        if (return_value)
        {
            if(distance_map_ != NULL)
            {
                distance_map_->update();
            }
            else
            {
                //ROS_INFO("No distance map available");
            }
        }
        return return_value;
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
        if(distance_map_ != NULL)
        {
            distance_map_->update();
        }
        else
        {
            //ROS_INFO("No distance map available");
        }
        return true;
    }
    else
    {
        //ROS_INFO("No octree available");
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
