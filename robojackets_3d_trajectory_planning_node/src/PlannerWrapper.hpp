#pragma once

#include <string>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory> // for make_unique
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "2dplane/2dplane.hpp"


class PlannerWrapper
{
public:
    PlannerWrapper();
    void step();

    std::shared_ptr<RRT::GridStateSpace> statespace_;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector3d>> birrt_;

    Eigen::Vector3d start_velocity_;
    Eigen::Vector3d goal_velocity_;
    std::vector<Eigen::Vector3d> previous_solution_;
};
