///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A 3d planning space representation for rt-rrt*
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_STRUCT_H_
#define JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_STRUCT_H_

#include "node_struct.h"
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

namespace js_trajectory_planning_node
{

struct PlanningSpace3d
{
    std::list<Node> node_space_;
    octomap::OcTree* octomap_space_;
    
    PlanningSpace3d()
        : octomap_space_(NULL)
    {
    }
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_STRUCT_H_
