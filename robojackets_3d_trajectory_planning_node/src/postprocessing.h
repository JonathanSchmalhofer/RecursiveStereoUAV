#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_POSTPROCESSING_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_POSTPROCESSING_H

#include "state_space.h"
#include <vector>

namespace RRT
{

/// @brief Reduce the vector to @max_size length
/// @details We do this by sampling to evenly distribute deletions
/// 
/// @param states     The vector of T values to sample
/// @param max_size Max length of the resulting vector
void DownSampleVector(std::vector<Eigen::Vector3d>& states, size_t max_size)
{
    if (states.size() > max_size)
    {
        int to_delete = states.size() - max_size;
        double spacing = (double)states.size() / (double)to_delete;
        double i = 0.0;
        while (to_delete)
        {
            to_delete--;
            states.erase(states.begin() + (int)(i + 0.5));
            i += spacing - 1.0;
        }
    }
}

/// @brief Deletes waypoints from @points
/// 
/// @param points A vector of states that constitutes the path
/// @param transitionValidator A function that returns a boolean indicating
/// whether or not a straight connection exists between the two given
/// states
void SmoothPath(std::vector<Eigen::Vector3d>& points, const StateSpace& state_space)
{
    int span = 2;
    while (span < points.size())
    {
        bool changed = false;
        for (int i = 0; i + span < points.size(); i++)
        {
            if (state_space.IsTransitionValid(points[i], points[i + span]))
            {
                for (int x = 1; x < span; x++)
                {
                    points.erase(points.begin() + i + 1);
                }
                changed = true;
            }
        }
        if (!changed)
            span++;
    }
}

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_POSTPROCESSING_H

