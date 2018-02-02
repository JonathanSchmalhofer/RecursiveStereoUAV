///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A 3d planning space representation for rrt
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_CLASS_H_

#include "node_class.h"
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

namespace js_trajectory_planning_node
{

class PlanningSpace3d
{
    
public:
    PlanningSpace3d()
        : octomap_space_(NULL)
    {
    }
    
    octomap::OcTree* GetPlanningSpace()
    {
        return planning_space_;
    }
    
private:
    octomap::OcTree* planning_space_;
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_PLANNING_SPACE_3D_STRUCT_H_
