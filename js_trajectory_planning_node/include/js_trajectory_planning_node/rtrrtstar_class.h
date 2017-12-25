///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RT-RRT* class
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_

namespace js_trajectory_planning_node
{

class RTRRTStarClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RTRRTStarClass();
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RTRRTStarClass();

private:
    /// @brief Expand and rewire tree
    void ExpandAndRewireTree();
    
    /// @brief Add node to tree
    void AddNodeToTree();
    
    /// @brief Rewire random nodes
    void RewireRandomNodes();
    
    /// @brief Rewire from tree root
    void RewireFromTreeRoot();
    
    /// @brief Plan a path for k steps
    void PlanPathForKSteps();
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
