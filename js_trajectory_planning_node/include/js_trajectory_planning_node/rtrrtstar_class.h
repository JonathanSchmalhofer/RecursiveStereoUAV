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
    
    /// @brief Perform a planning cycle
    void PerformPlanningCycleOnce();
    
    /// @brief Update the new agent location
    void UpdateXAgent(Node x_agent);
    
    /// @brief Update the new goal location
    void UpdateXGoal(Node x_goal);
    
    /// @brief Update the obstacles (planning) space
    void UpdateXiObs();
    
    /// @brief Update the free (planning) space
    void UpdateXiFree();

private:
    /// @brief Expand and rewire tree
    void ExpandAndRewireTree();
    
    /// @brief Add node to tree
    void AddNodeToTree();
    
    /// @brief Rewire random nodes
    void RewireRandomNodes();
    
    /// @brief Rewire from tree root
    void RewireFromTreeRoot();
    
    /// @brief Find nodes near to a specific node (in 3d)
    std::list<std::reference_wrapper<Node>> FindNodesNear3d(Node x_in);
    
    /// @brief Calculate cost for node x_in (depending on all ancestors, but they will not be updated)
    double cost(Node& x_in);
    
    /// @brief Get the (current) volume of the search space (in paper: mue(Xi))
    double GetVolumeOfSearchSpace3d();
    
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
    
    /// @brief Perform random sampling in planning space and return a node
    Node SampleRandom();
    
    /// @brief Samples randomly in the line between x_in and the node of the tree that is closest to x_in
    Node LineTo(Node x_in);
    
    /// @brief Sample the entire planning space uniformly
    Node Uniform();
    
    /// @brief Sample the entire planning space uniformly
    Node Ellipsoid(Node x_a, Node x_b);
    
    /// @brief Draw a 3d point within the ellipsoid defined by a center point and covariance matrix
    Vector3d DrawWithinEllipsoid(const Eigen::Matrix3d covariance, const Eigen::Vector3d center);
    
    /// @brief Generate a uniform random number (double) between a and b. Note: b must be greater than a
    double UniformRandomNumberBetween(double a, double b);
    
    /// @brief Generate a normally distributed random number
    double NormalRandomNumber();
    
    /// @brief Uniform sampling along the line between x_in, which does not have to be part of the tree, and the node of the tree closest to it.
    Node& GetClosestNodeInTree(Node x_in);
    
    /// @brief Check whether the line between x_a and x_b is collision free
    bool CheckIfCollisionFreeLineBetween(Node x_a, Node x_b);
    
    /// @brief Calculate euclidian distance in 3d between the nodes a and b
    double EuclidianDistance3d(Node a, Node b);
    
    /// @brief Holds the count of expansion and rewiring attempts per loop cycle
    std::uint32_t counter_expansions_and_rewiring_;
    
    /// @brief Holds the current node at which the agent is located
    Node x_agent;
    
    /// @brief Holds the current goal node
    Node x_goal;
    
    /// @brief Holds the current root node
    Node x_0;
    
    /// @brief Holds the currently known (planning) space of all obstacles
    PlanningSpace3d Xi_obs;
    
    /// @brief Queue holding nodes for random rewiring
    std::list<Node> Q_r;
    
    /// @brief Queue holding nodes for rewiring from the tree root
    std::list<Node> Q_s;
    
    /// @brief The search tree
    std::list<Node> T;
    
    /// @brief Maximum iterations to perform per cycle step
    const std::uint32_t kmax_number_expansions_and_rewiring = 300;
    
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
    
    /// @brief Maximum number of neighbours to be considered (k_max)
    const std::uint32_t kmaximum_number_closest_neighbours = 10;
    
    /// @brief Maximum radius to find neighbours to be considered (r_s)
    const double kradius_closest_neighbours = 1.0;
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_RTRRTSTAR_CLASS_H_
