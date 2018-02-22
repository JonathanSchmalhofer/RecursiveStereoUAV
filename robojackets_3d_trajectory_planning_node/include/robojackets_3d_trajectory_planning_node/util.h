#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_UTIL_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_UTIL_H

#include <Eigen/Dense>

namespace RRT
{

template <typename T>
bool IsInRange(T n, T min, T max)
{
    return (n >= min) && (n <= max);
}

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_UTIL_H
