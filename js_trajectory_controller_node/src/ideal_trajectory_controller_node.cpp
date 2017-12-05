///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to control the trajectory from trajectory planner for an ideal dynamics model.
///
#include <string>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ideal_trajectory_controller_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    while (ros::ok())
    {
    }

    return 0;
}
