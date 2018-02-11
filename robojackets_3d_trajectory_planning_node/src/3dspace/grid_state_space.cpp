#include <math.h>
#include "3dspace/grid_state_space.h"
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
    return SpaceStateSpace::IsStateValid(point) &&
           !obstacle_grid_.IsObstacleAt(obstacle_grid_.GetGridSquareForLocation(point));
}

Vector3d GridStateSpace::GetIntermediateState(const Vector3d& source,
                                              const Vector3d& target,
                                              double min_step_size,
                                              double max_step_size) const
{
    bool debug = false;

    Vector3d delta = target - source;
    delta = delta / delta.norm();  //  unit vector
    double dist = obstacle_grid_.GetDistanceToNearestObstacle(source, max_step_size * 2);

    double step_size =
        (dist / max_step_size) *
        min_step_size;  // scale based on how far we are from obstacles
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

    Vector3d delta = to - from;

    //  get the corners of this segment in integer coordinates.  This limits our
    //  intersection test to only the boxes in that square
    Vector3i discrete_from = obstacle_grid_.GetGridSquareForLocation(from);
    Vector3i discrete_to = obstacle_grid_.GetGridSquareForLocation(to);
    int x1 = discrete_from.x(), y1 = discrete_from.y(), z1 = discrete_from.z();
    int x2 = discrete_to.x(), y2 = discrete_to.y(), z2 = discrete_to.z();

    //  order ascending
    if (x1 > x2) swap<int>(x1, x2);
    if (y1 > y2) swap<int>(y1, y2);
    if (z1 > z2) swap<int>(z1, z2);

    double grid_square_width = GetWidth() / obstacle_grid_.GetDiscretizedWidth();
    double grid_square_height = GetHeight() / obstacle_grid_.GetDiscretizedHeight();
    double grid_square_depth = GetDepth() / obstacle_grid_.GetDiscretizedDepth();

    //  check all cubes from (x1, y1, z1) to (x2, y2, z2)
    for (int x = x1; x <= x2; x++)
    {
        for (int y = y1; y <= y2; y++)
        {
            for (int z = z1; z <= z2; z++)
            {
                if (obstacle_grid_.IsObstacleAt(x, y, z))
                {
                    //  there's an obstacle here, so check for intersection

                    //  the corners of this obstacle square
                    Vector3d upper_left_corner(x * grid_square_width, y * grid_square_height, z * grid_square_depth);
                    Vector3d boundary_corner(upper_left_corner.x() + grid_square_width,
                                      upper_left_corner.y() + grid_square_height,
                                      upper_left_corner.z() + grid_square_depth);

                    if (delta.x() != 0)
                    {
                        /**
                         * Find slope and y-intercept of the line passing through
                         * @from and
                         * @to.
                         * y1 = m*x1+b
                         * b = y1-m*x1
                         */
                        double slope = delta.y() / delta.x();
                        double b = to.y() - to.x() * slope;

                        /*
                         * First check intersection with the vertical segments of
                         * the box.
                         * Use y=mx+b for the from-to line and plug in the x value
                         * for each
                         * wall
                         * If the corresponding y-value is within the y-bounds of
                         * the vertical
                         * segment, it's an intersection.
                         */
                        double y_intersection = slope * upper_left_corner.x() + b;
                        if (IsInRange<double>(y_intersection, upper_left_corner.y(), boundary_corner.y()))
                            return false;
                        y_intersection = slope * boundary_corner.x() + b;
                        if (IsInRange<double>(y_intersection, upper_left_corner.y(), boundary_corner.y()))
                            return false;

                        /*
                         * Check intersection with horizontal sides of box
                         * y = k;
                         * y = mx+b;
                         * mx+b = k;
                         * mx = k - b;
                         * (k - b) / m = x;  is x within the horizontal range of the
                         * box?
                         */
                        if (slope == 0) return false;
                        double x_intersection = (upper_left_corner.y() - b) / slope;
                        if (IsInRange<double>(x_intersection, upper_left_corner.x(), boundary_corner.x()))
                            return false;
                        x_intersection = (boundary_corner.y() - b) / slope;
                        if (IsInRange<double>(x_intersection, upper_left_corner.x(), boundary_corner.x()))
                            return false;
                    }
                    else
                    {
                        //  vertical line - slope undefined

                        //  see if it's within the x-axis bounds of this obstacle
                        //  box
                        if (IsInRange<double>(from.x(), upper_left_corner.x(), boundary_corner.x()))
                        {
                            //  order by y-value
                            //  note: @lower has a smaller value of y, but will
                            //  appear higher
                            //  visually on the screen due to qt's coordinate layout
                            Vector3d lower(from);
                            Vector3d higher(to);
                            if (higher.y() < lower.y())
                                swap<Vector3d>(lower, higher);

                            //  check for intersection based on y-values
                            if (lower.y() < upper_left_corner.y() &&
                                higher.y() > upper_left_corner.y())
                                return false;
                            if (lower.y() < boundary_corner.y() &&
                                higher.y() > boundary_corner.y())
                                return false;
                        }
                    }
                }
            }
        }
    }
    return true;
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
