#include "PlannerWrapper.h"
#include "planning/Path.h"

PlannerWrapper::PlannerWrapper()
{
    Eigen::Vector3d size(800, 800, 800);
    statespace_ = std::make_shared<RRT::GridStateSpace>(size.x(), size.y(), size.z(), 40, 30, 30);
    birrt_ = std::make_unique<RRT::BiRRT<Eigen::Vector3d>>(statespace_, 3);

    //  setup birrt
    birrt_->setStartState(size / 10);
    birrt_->setGoalState(size / 2);
    birrt_->setMaxStepSize(100);
    birrt_->setStepSize(10);
    birrt_->setMaxIterations(1000);
    birrt_->setASCEnabled(true);
    birrt_->setGoalBias(0.2);
    birrt_->setWaypointBias(0.2);
    birrt_->setGoalMaxDist(5);
    

    start_velocity_ = Eigen::Vector3d(3, 0, 0);
    goal_velocity_ = Eigen::Vector3d(0, 3, 0);
}

void PlannerWrapper::step()
{
    int numTimes = 5;

    for (int i = 0; i < numTimes; i++)
    {
        birrt_->grow();
    }

    // store solution
    previous_solution_.clear();
    if (birrt_->startSolutionNode() != nullptr)
    {
        //ROS_INFO("   Found a solution");
        previous_solution_ = birrt_->getPath();
        RRT::SmoothPath(previous_solution_, *statespace_);
    }
}
