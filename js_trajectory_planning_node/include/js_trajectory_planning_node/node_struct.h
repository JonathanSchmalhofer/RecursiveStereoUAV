///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A node struct representation for rt-rrt*
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_
#define JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_

#include <cstddef>      // NULL
#include <list>         // std::list
#include <limits>       // std::numeric_limits

namespace js_trajectory_planning_node
{
    
struct Vector3d
{
    double x_;
    double y_;
    double z_;
    
    Vector3d(double x = 0.0f, double y = 0.0f, double z = 0.0f)
        : x_(0.0f), y_(0.0f), z_(0.0f)
    {
        x_ = x;
        y_ = y;
        z_ = z;
    };
};

struct NodeData
{
    bool active_;
    Vector3d position_;
    double cost_to_start_;    
    
    NodeData(Vector3d position = Vector3d(0.0f, 0.0f, 0.0f), double cost_to_start = std::numeric_limits<double>::infinity(), bool active = false)
        : active_(false),
          position_(0.0f, 0.0f, 0.0f),
          cost_to_start_(std::numeric_limits<double>::infinity())
    {
        active_ = active;
        position_ = position;
        cost_to_start_ = cost_to_start;
    }
};

}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_
