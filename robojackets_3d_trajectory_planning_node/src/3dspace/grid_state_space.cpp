#include <math.h>
#include "robojackets_3d_trajectory_planning_node/3dspace/grid_state_space.h"
#include "util.h"
#include <stdexcept>

using namespace Eigen;
using namespace std;

#include <iostream>

namespace RRT
{

GridStateSpace::GridStateSpace(double width,
                               double height,
                               double depth,
                               int discretized_width,
                               int discretized_height,
                               int discretized_depth)
    : SpaceStateSpace(width, height, depth),
      obstacle_grid_(width, height, depth, discretized_width, discretized_height, discretized_depth)
{
}

bool GridStateSpace::IsStateValid(const Vector3d& point) const
{
    /*return SpaceStateSpace::IsStateValid(point) &&
           !obstacle_grid_.IsObstacleAt(obstacle_grid_.GetGridSquareForLocation(point));*/
    return (SpaceStateSpace::IsStateValid(point) &&
            !obstacle_grid_.IsObstacleAt(point));
}

Vector3d GridStateSpace::GetIntermediateState(const Vector3d& source,
                                              const Vector3d& target,
                                              double min_step_size,
                                              double max_step_size) const
{
    bool debug = false;

    Vector3d delta = target - source;
    delta = delta / delta.norm();  //  unit vector
    double max_search_radius = 2 * max_step_size;
    double dist = obstacle_grid_.GetDistanceToNearestObstacle(source, max_search_radius);

    // no obstacle in search radius - make distance random with k * dist
    // with k within [0.5;1.0]
    if (dist == max_search_radius)
    {
        double scale_factor = (drand48() / 2) + 0.5;
        dist *= scale_factor;
    }
    double step_size = (dist / max_step_size) * min_step_size;  // scale based on how far we are from obstacles
    if (step_size > max_step_size) step_size = max_step_size;
    if (step_size < min_step_size) step_size = min_step_size;
    if (debug)
    {
        cout << "ASC intermediate state" << endl;
        cout << "  stepsize: " << min_step_size << endl;
        cout << "  nearest obs dist: " << dist << endl;
        cout << "  maximum stepsize: " << max_step_size << endl;
        cout << "  new step: " << step_size << endl;
    }

    Vector3d val = source + delta * step_size;
    return val;
}

bool GridStateSpace::IsTransitionValid(const Vector3d& from,
                                       const Vector3d& to) const
{
    //  make sure we're within bounds
    if (!IsStateValid(to)) return false;

    return obstacle_grid_.CheckIfCollisionFreeLineBetween(from, to);
}

const ObstacleGrid& GridStateSpace::GetObstacleGrid() const
{
    return obstacle_grid_;
}

ObstacleGrid& GridStateSpace::GetObstacleGrid()
{
    return obstacle_grid_;
}

}  // namespace RRT
