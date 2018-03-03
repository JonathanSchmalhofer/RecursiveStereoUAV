#include "js_trajectory_planning_node/planner_wrapper.h"
#include "js_trajectory_planning_node/postprocessing.h"

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


    // add test wall
    ROS_INFO("Make Octomap great again");
    double x_wall, y_wall, z_wall;
    y_wall = 400;
    for(x_wall = 0; x_wall <= size.x(); x_wall += 1)
    {
        for(z_wall = 0; z_wall <= size.z(); z_wall += 1)
        {
            Eigen::Vector3d wall_point(x_wall, y_wall, z_wall);
            statespace_->GetObstacleGrid().InsertOccupiedMeasurement(wall_point);
        }
    }
    ROS_INFO("Trump was here");
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

Eigen::Vector3d PlannerWrapper::GetStartVelocity() { return start_velocity_; };
Eigen::Vector3d PlannerWrapper::GetGoalVelocity() { return goal_velocity_; };
std::vector<Eigen::Vector3d> PlannerWrapper::GetPreviousSolution() { return previous_solution_; };
