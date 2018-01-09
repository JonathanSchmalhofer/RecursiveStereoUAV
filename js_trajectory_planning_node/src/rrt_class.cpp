///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RRT class based on https://github.com/RoboJackets/rrt
///
#include "js_trajectory_planning_node/rrt_class.h"

namespace js_trajectory_planning_node
{

RRTClass::RRTClass()
{
    Initialize();
    FinalizeLoopCycle();
}

RRTClass::~RRTClass()
{
    Initialize();
}

void RRTClass::Initialize()
{
}

void RRTClass::PerformPlanningCycleOnce()
{
    while(IsTimeLeftForExpansionAndRewiring())
    {
    }
    FinalizeLoopCycle();
}

bool RRTClass::IsTimeLeftForExpansionAndRewiring()
{
    bool is_time_left = false;
    if (counter_expansions_and_rewiring_ < kmax_number_expansions_and_rewiring)
    {
        is_time_left = true;
    }
    
    // Todo: add another iterator limit (e.g. check for time)
    return is_time_left;
}

void RRTClass::FinalizeLoopCycle()
{
    counter_expansions_and_rewiring_ = 0;
}

}  // namespace js_trajectory_planning_node
