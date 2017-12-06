///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to control the trajectory from trajectory planner for an ideal dynamics model.
///
#include <string>
#include <ros/ros.h>
#include <js_trajectory_controller_node/trajectory_point_to_airsim_class.h>
#include <js_messages/Trajectory3D.h>
#include <js_messages/Trajectory3DPointStampedRoot_generated.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ideal_trajectory_controller_node");
    ros::NodeHandle node_handle;
    
    js_trajectory_controller_node::Trajectory3DPointToAirSimClass("tcp://*:5676");
    
    ROS_INFO("Starting node");

    while (ros::ok())
    {
    }

    return 0;
}
