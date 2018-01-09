///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A node representation based on https://github.com/RoboJackets/rrt/blob/master/src/rrt/Tree.hpp
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_NODE_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_NODE_CLASS_H_

#include <cstdint>      // std::uint32_t
#include <cstddef>      // NULL
#include <vector>       // std::vector
#include <list>         // std::list

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

class Node3d
{
public:
    Node3d(const Vector3d& position, Node3d* parent = nullptr)
        : position_(position)
    {
        parent_ = parent;
        if(parent_)
        {
            parent_->children_.push_back(this);
        }
    }

    const Node3d* GetParent() const
    {
        return parent_;
    }

    /// @brief Gets the number of ancestors (parent, parent's parent, etc) that the node has. Returns 0 if it doesn't have a parent.
    std::uint32_t GetDepth() const
    {
        std::uint32_t n = 0;
        for (   Node3d* ancestor = parent_;
                ancestor != nullptr;
                ancestor = ancestor->parent_)
        {
            n++;
        }
        return n;
    }

    /// @brief Gets the position of the node in 3d space.
    Vector3d GetPosition() const
    {
        return position_;
    }

private:
    Vector3d position_;
    std::list<Node3d*> children_;
    Node3d* parent_;
};

}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_NODE_CLASS_H_
