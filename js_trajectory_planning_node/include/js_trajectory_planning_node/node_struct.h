///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A node struct representation for rt-rrt*
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_
#define JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_

namespace js_trajectory_planning_node
{
    
struct Vector3d
{
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    
    Vector3d(double x = 0, double y = 0, double z = 0)
    {
        x_ = x;
        y_ = y;
        z_ = z;
    }
}

struct Node
{
    bool active_ = true;
    Vector3d position_;
    double cost_to_start_ = std::numeric_limits<double>::infinity();
    
    Node *parent_of_parent_ = NULL;
    Node *parent_ = NULL;
    std::list<Node*> children_;
    
    Nodes(Vector3d position, cost_to_start, Node *parent = NULL)
    {
        position_ = position;
        cost_to_start_ = cost_to_start;
        parent_ = parent;
    }
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_NODE_STRUCT_H_
