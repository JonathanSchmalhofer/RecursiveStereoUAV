#include "planner_wrapper.h"
#include "postprocessing.h"

PlannerWrapper::PlannerWrapper()
{
    Eigen::Vector3d size(800, 800, 800);
    statespace_ = std::make_shared<RRT::GridStateSpace>(size.x(), size.y(), size.z(), 40, 40, 40);
    birrt_ = std::make_unique<RRT::BiRRT<Eigen::Vector3d>>(statespace_, 3);

    Eigen::Vector3d start_state(10, 10, 10);
    Eigen::Vector3d goal_state(790, 790, 790);
    //  setup birrt
    birrt_->SetStartState(start_state);
    birrt_->SetGoalState(goal_state);
    birrt_->SetMaxStepSize(100);
    birrt_->SetStepSize(10);
    birrt_->SetMaxIterations(1000);
    birrt_->SetAdaptiveScalingEnable(true);
    birrt_->SetGoalBias(0.2);
    birrt_->SetWaypointBias(0.2);
    birrt_->SetMaxDistanceToGoal(5);

    start_velocity_ = Eigen::Vector3d(3, 0, 0);
    goal_velocity_ = Eigen::Vector3d(0, 3, 0);
}

void PlannerWrapper::Step()
{
    int number_step_iterations = 5;

    for (int i = 0; i < number_step_iterations; i++)
    {
        birrt_->Grow();
    }

    // store solution
    previous_solution_.clear();
    if (birrt_->GetStartSolutionNode() != nullptr)
    {
        ROS_INFO("   Found a solution");
        previous_solution_ = birrt_->GetPath();
        RRT::SmoothPath(previous_solution_, *statespace_);
    }
}
