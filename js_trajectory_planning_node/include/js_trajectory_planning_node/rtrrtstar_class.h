///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RT-RRT* class
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_

#include "planning_space_3d_struct.h"
#include <random>
#include <cmath>
#include <algorithm>    // std::sort
#include <functional>   // std::reference_wrapper
#include <Eigen/Dense>

namespace js_trajectory_planning_node
{

class RTRRTStarClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RTRRTStarClass();
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RTRRTStarClass();
    
    /// @brief Initialize tree T, queue Q_r and queue Q_s as well
    void Initialize();
    
    /// @brief Perform a planning cycle
    void PerformPlanningCycleOnce();
    
    /// @brief Update the new agent location
    void UpdateXAgent(NodeIt x_in);
    
    /// @brief Update the new goal location
    void UpdateXGoal(NodeIt x_in);
    
    /// @brief Update the new tree root location
    void UpdateX0(NodeIt x_in);
    
    /// @brief Update the obstacles (planning) space
    void UpdateXiObs();

private:
    /// @brief Save position of x_in into a string
    std::string ToString(const NodeIt& x_in);
    
    /// @brief Print node x_in using ROS_INFO indented as often as denoted by level // TODO: delete for decoupling from ROS
    void IndentedRosInfo(const tree<NodeData>& tree_in);
    
    /// @brief Print list of nodes list_in using ROS_INFO // TODO: delete for decoupling from ROS
    void NodeListRosInfo(const std::list<NodeIt>& list_in);
    
    /// @brief Expand and rewire tree
    void ExpandAndRewireTree();
    
    /// @brief Add node to tree and return the iterator to the new entry
    NodeIt AddNodeToTree(NodeIt& x_new, const NodeIt& x_closest, std::list<NodeIt> Xi_near);
    
    /// @brief Rewire random nodes
    void RewireRandomNodes();
    
    /// @brief Rewire from tree root
    void RewireFromTreeRoot();
    
    /// @brief Find nodes near to a specific node (in 3d)
    std::list<NodeIt> FindNodesNear3d(const NodeIt& x_in);

    /// @brief Calculate cost for node x_in (depending on all ancestors, but they will not be updated)
    double cost(const NodeIt& x_in);
    
    /// @brief Get the (current) volume of the search space (in paper: mue(Xi))
    double GetVolumeOfSearchSpace3d();
    
    /// @brief Plan a path for k steps
    void PlanPathForKSteps();
    
    /// @brief Cleanup inactive nodes and delete them from T
    void CleanupInactiveNodes();
    
    /// @brief Determine whether there is time left for expansion and rewiring
    bool IsTimeLeftForExpansionAndRewiring();
    
    /// @brief Determine whether there is time left for rewiring random nodes
    bool IsTimeLeftForRewireRandomNodes();
    
    /// @brief Determine whether there is time left for rewiring from the tree root
    bool IsTimeLeftForRewireFromTreeRoot();
    
    /// @brief Determine whether there is time left for cleaning up inactive nodes
    bool IsTimeLeftForCleanupInactiveNodes();
    
    /// @brief Check whether the agent is close to or at the tree root
    bool IsAgentCloseToTreeRoot();
    
    /// @brief Change the tree root node to the next immediate node
    void ChangeTreeRootToNextImmediateNode();
    
    /// @brief Convenience function to reset internal states at the end of the loop cycle
    void FinalizeLoopCycle();
    
    /// @brief Perform random sampling in planning space and return a node
    NodeIt SampleRandom();
    
    /// @brief Samples randomly in the line between x_in and the node of the tree that is closest to x_in
    NodeIt LineTo(const NodeIt& x_in);
    
    /// @brief Sample the entire planning space uniformly
    NodeIt Uniform();
    
    /// @brief Sample the entire planning space uniformly
    NodeIt Ellipsoid(const NodeIt& x_a, const NodeIt& x_b);
    
    /// @brief Draw a 3d point within the ellipsoid defined by a center point and covariance matrix
    Vector3d DrawWithinEllipsoid(const Eigen::Matrix3d covariance, const Eigen::Vector3d center);
    
    /// @brief Generate a uniform random number (double) between a and b. Note: b must be greater than a
    double UniformRandomNumberBetween(const double a, const double b);
    
    /// @brief Generate a normally distributed random number
    double NormalRandomNumber();
    
    /// @brief Uniform sampling along the line between x_in, which does not have to be part of the tree, and the node of the tree closest to it.
    NodeIt GetClosestNodeInTree(const NodeIt& x_in);
    
    /// @brief Check whether the line between x_a and x_b is collision free
    bool CheckIfCollisionFreeLineBetween(const NodeIt& x_a, const NodeIt& x_b);
    
    /// @brief Calculate euclidian distance in 3d between the nodes a and b
    double EuclidianDistance3d(const NodeIt& a, const NodeIt& b);
    
    /// @brief Changes the parent of child_node in T to new_parent (including all children from child_node)
    NodeIt ChangeParent(NodeIt& child_node, const NodeIt& new_parent);
    
    /// @brief Holds the count of expansion and rewiring attempts per loop cycle
    std::uint32_t counter_expansions_and_rewiring_;
    
    /// @brief Holds the count of rewiring random nodes attempts per loop cycle
    std::uint32_t counter_rewire_random_nodes_;
    
    /// @brief Holds the count of rewiring from the tree root attempts per loop cycle
    std::uint32_t counter_rewire_from_tree_root_;
    
    /// @brief Holds the count of how many inactive nodes have been deleted per loop cycle
    std::uint32_t counter_cleanup_inactive_nodes_;
    
    /// @brief Holds the current node at which the agent is located
    NodeIt x_agent;
    
    /// @brief Holds the current goal node
    NodeIt x_goal;
    
    /// @brief Holds the current root node
    NodeIt x_0;
    
    /// @brief Holds the currently known (planning) space of all obstacles (Note: Xi_free is included here, as octomaps are being used)
    PlanningSpace3d Xi_obs;
    
    /// @brief Queue holding nodes for random rewiring
    std::list<NodeIt> Q_r;
    
    /// @brief Queue holding nodes for rewiring from the tree root
    std::list<NodeIt> Q_s;
    
    /// @brief The search tree
    tree<NodeData> T;
    
    /// @brief Maximum iterations to perform per cycle step
    const std::uint32_t kmax_number_expansions_and_rewiring = 5;
    
    /// @brief Maximum iterations to rewire random nodes (per cycle step)
    const std::uint32_t kmax_number_rewire_random_nodes = 5;
    
    /// @brief Maximum iterations to rewire from the tree root (per cycle step)
    const std::uint32_t kmax_number_rewire_from_tree_root = 5;
    
    /// @brief Maximum iterations to cleanup inactive nodes (per cycle step)
    const std::uint32_t kmax_number_cleanup_inactive_nodes = 20;
    
    /// @brief User given constant for random sampling
    const double kalpha = 0.1;
    
    /// @brief User given constant for random sampling for dividing between uniform sampling and sampling within a ellpsoid
    const double kbeta = 1.4;
    
    /// @brief Minimum extent in x-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_x = 100;
    
    /// @brief Minimum extent in y-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_y = 100;
    
    /// @brief Minimum extent in z-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_z = 20;
    
    /// @brief Maximum number of nodes allowed in the tree T
    const std::uint32_t kmaximum_number_nodes_in_tree = 5000;
    
    /// @brief Maximum number of neighbours to be considered (k_max)
    const std::uint32_t kmaximum_number_closest_neighbours = 100;
    
    /// @brief Maximum radius to find neighbours to be considered (r_s)
    const double kradius_closest_neighbours = 10.0;
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
