///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RT-RRT* class based on https://github.com/rishabh1b/RealTimePathPlanning/
///
#include "js_trajectory_planning_node/rtrrtstar_class.h"

namespace js_trajectory_planning_node
{

RTRRTStarClass::RTRRTStarClass()
{
    counter_expansions_and_rewiring_ = 0;
}

RTRRTStarClass::~RTRRTStarClass()
{
}

void RTRRTStarClass::PerformPlanningCycleOnce()
{
    // Todo: Update(x_goal, x_a, Xi_free and Xi_obs)
    while(IsTimeLeftForExpansionAndRewiring())
    {
        ExpandAndRewireTree();
    }
    PlanPathForKSteps();
    if(IsAgentCloseToTreeRoot())
    {
        ChangeTreeRootToNextImmediateNode();
    }
    FinalizeLoopCycle();
}

void RTRRTStarClass::ExpandAndRewireTree()
{
    // Todo: Sample x_rand
    // Todo: get closest node arg min dist(x,x_rand)
    if(true) // Todo: check if line(x_closest,x_rand) is in freespace Xi_free
    {
        FindNodesNear();
        if(true) // Todo: check line 6 Algorithm 2
        {
            AddNodeToTree();
            // Todo: Push x_rand to the first of Q_r
        }
        else
        {
            // Todo: Push x_closest to the first of Q_r
        }
        RewireRandomNodes();
    }
    RewireFromTreeRoot();
    ++counter_expansions_and_rewiring_;
}

void RTRRTStarClass::RewireRandomNodes()
{
}

void RTRRTStarClass::RewireFromTreeRoot()
{
}

void RTRRTStarClass::FindNodesNear()
{
}

void RTRRTStarClass::AddNodeToTree()
{
}

void RTRRTStarClass::PlanPathForKSteps()
{
}

bool RTRRTStarClass::IsTimeLeftForExpansionAndRewiring()
{
    bool is_time_left = false;
    if (counter_expansions_and_rewiring_ < kmax_number_expansions_and_rewiring)
    {
        is_time_left = true;
    }
    return is_time_left;
}

bool RTRRTStarClass::IsAgentCloseToTreeRoot()
{
    bool is_close = false;
    
    return is_close;
}

void RTRRTStarClass::ChangeTreeRootToNextImmediateNode()
{
}

void RTRRTStarClass::FinalizeLoopCycle()
{
    counter_expansions_and_rewiring_ = 0;
}

}  // namespace js_trajectory_planning_node
