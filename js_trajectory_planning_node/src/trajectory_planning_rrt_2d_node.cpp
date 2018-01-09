///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to calculate the trajectory to be followed.
///
#include <string>
#include <ros/ros.h>
#include <js_messages/Trajectory3D.h>
#include "js_trajectory_planning_node/rrt_class.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planning_rrt_2d_node");
    
    ros::NodeHandle node_handle;
    ros::Publisher trajectory3d_publisher = node_handle.advertise<js_messages::Trajectory3D>("/trajectory_planning/trajectory3d", 1000);
    //js_trajectory_planning_node::RRTClass planner;
    
    ROS_INFO("Starting trajectory planning node");    
    
    ros::Rate node_rate(2.0f); // 2.0 hz
    while (ros::ok())
    {
        //ROS_INFO("Planning step");
        //planner.PerformPlanningCycleOnce();

        //ROS_INFO("Publish step");
        js_messages::Trajectory3D trajectory3d_message;
        trajectory3d_publisher.publish(trajectory3d_message);
        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;
}
