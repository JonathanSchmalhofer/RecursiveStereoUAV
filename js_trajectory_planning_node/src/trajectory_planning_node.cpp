///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to calculate the trajectory to be followed.
///
#include <string>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planning_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    ros::Rate node_rate(25.0f); // 25.0 hz
    while (ros::ok())
    {
        ros::spinOnce();
        node_rate.sleep();
    }

    return 0;
}
