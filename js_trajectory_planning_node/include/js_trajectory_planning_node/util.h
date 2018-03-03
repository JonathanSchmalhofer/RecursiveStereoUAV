#ifndef JS_TRAJECTORY_PLANNING_NODE_UTIL_H
#define JS_TRAJECTORY_PLANNING_NODE_UTIL_H

#include <Eigen/Dense>

namespace RRT
{

template <typename T>
bool IsInRange(T n, T min, T max)
{
    return (n >= min) && (n <= max);
}

}  // namespace RRT

#endif // JS_TRAJECTORY_PLANNING_NODE_UTIL_H
