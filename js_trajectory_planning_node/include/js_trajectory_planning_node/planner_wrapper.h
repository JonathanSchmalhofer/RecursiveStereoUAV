#ifndef JS_TRAJECTORY_PLANNING_NODE_PLANNERWRAPPER_H
#define JS_TRAJECTORY_PLANNING_NODE_PLANNERWRAPPER_H

#include <string>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory> // for make_unique
#include "3dspace/grid_state_space.h"
#include "birrt.h"
#include "3dspace/3dspace.h"


class PlannerWrapper
{
public:
    PlannerWrapper(Eigen::Vector3d size, Eigen::Vector3d start, Eigen::Vector3d goal);
    void Step();
    void Reset();

    Eigen::Vector3d GetStartVelocity();
    Eigen::Vector3d GetGoalVelocity();
    void SetStartState(Eigen::Vector3d state);
    void SetGoalState(Eigen::Vector3d state);
    std::vector<Eigen::Vector3d> GetPreviousSolution();

    std::shared_ptr<RRT::GridStateSpace> statespace_;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector3d>> birrt_;

private:
    Eigen::Vector3d start_state_;
    Eigen::Vector3d goal_state_;
    Eigen::Vector3d start_velocity_;
    Eigen::Vector3d goal_velocity_;
    std::vector<Eigen::Vector3d> previous_solution_;
};

#endif // JS_TRAJECTORY_PLANNING_NODE_PLANNERWRAPPER_H
