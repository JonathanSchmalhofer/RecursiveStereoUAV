///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RT-RRT* class
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_

#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>

namespace js_trajectory_planning_node
{

class RTRRTStarClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RTRRTStarClass();
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RTRRTStarClass();
    
    /// @brief Perform a planning cycle
    void PerformPlanningCycleOnce();

private:
    /// @brief Expand and rewire tree
    void ExpandAndRewireTree();
    
    /// @brief Add node to tree
    void AddNodeToTree();
    
    /// @brief Rewire random nodes
    void RewireRandomNodes();
    
    /// @brief Rewire from tree root
    void RewireFromTreeRoot();
    
    /// @brierf Find nodes near to a specific node
    void FindNodesNear();
    
    /// @brief Plan a path for k steps
    void PlanPathForKSteps();
    
    /// @brief Determine whether there is time left for expansion and rewiring
    bool IsTimeLeftForExpansionAndRewiring();
    
    /// @brief Check whether the agent is close to or at the tree root
    bool IsAgentCloseToTreeRoot();
    
    /// @brief Change the tree root node to the next immediate node
    void ChangeTreeRootToNextImmediateNode();
    
    /// @brief Convenience function to reset internal states at the end of the loop cycle
    void FinalizeLoopCycle();
    
    /// @brief Holds the count of expansion and rewiring attempts per loop cycle
    std::uint32_t counter_expansions_and_rewiring_;
    
    const std::uint32_t kmax_number_expansions_and_rewiring = 300;
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
