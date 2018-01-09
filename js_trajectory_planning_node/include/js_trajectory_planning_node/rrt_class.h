///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RRT class
///
#ifndef JS_TRAJECTORY_PLANNING_RRT_2D_NODE_RRT_CLASS_H_
#define JS_TRAJECTORY_PLANNING_RRT_2D_NODE_RRT_CLASS_H_

#include "node_class.h"
#include <random>
#include <cmath>
#include <algorithm>    // std::sort
#include <functional>   // std::reference_wrapper
#include <Eigen/Dense>

namespace js_trajectory_planning_node
{

class RRTClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RRTClass();
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RRTClass();
    
    /// @brief Initialize tree T, queue Q_r and queue Q_s as well
    void Initialize();
    
    /// @brief Perform a planning cycle
    void PerformPlanningCycleOnce();

private:    
    /// @brief Determine whether there is time left for expansion and rewiring
    bool IsTimeLeftForExpansionAndRewiring();
    
    /// @brief Convenience function to reset internal states at the end of the loop cycle
    void FinalizeLoopCycle();
    
    /// @brief Holds the count of expansion and rewiring attempts per loop cycle
    std::uint32_t counter_expansions_and_rewiring_;
    
    /// @brief Maximum iterations to perform per cycle step
    const std::uint32_t kmax_number_expansions_and_rewiring = 5;
    
    /// @brief Minimum extent in x-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_x = 100;
    
    /// @brief Minimum extent in y-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_y = 100;
    
    /// @brief Minimum extent in z-direction to be considered for uniform sampling in planning space
    const double kminimum_uniform_extent_z = 0;
};
}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_RRT_2D_NODE_RRT_CLASS_H_
