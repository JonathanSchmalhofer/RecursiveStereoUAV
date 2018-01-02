///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to calculate the trajectory to be followed.
///
#include <string>
#include <ros/ros.h>
#include <js_messages/Trajectory3D.h>
#include "js_trajectory_planning_node/rtrrtstar_class.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planning_node");
    
    ros::NodeHandle node_handle;
    ros::Publisher trajectory3d_publisher = node_handle.advertise<js_messages::Trajectory3D>("/trajectory_planning/trajectory3d", 1000);
    js_trajectory_planning_node::RTRRTStarClass planner;
    
    ROS_INFO("Starting trajectory planning node");

    /*
    planner.IndentedRosInfo(planner.T);

    ////// x_a
    ROS_INFO("------------------------------------------------------------------");
    ROS_INFO("T.size() = %zd", planner.T.size());
    ROS_INFO("Adding 1 node");
    js_trajectory_planning_node::NodeIt x_a = planner.T.insert(planner.T.begin(),js_trajectory_planning_node::NodeData(js_trajectory_planning_node::Vector3d(1, 0, 0)));
    ROS_INFO("T.size() = %zd", planner.T.size());

    ROS_INFO(std::string("Sampled: " + planner.ToString(x_a)).c_str());
    js_trajectory_planning_node::NodeIt x_closest = planner.GetClosestNodeInTree(x_a);
    ROS_INFO(std::string("Closest: " + planner.ToString(x_closest)).c_str());

    std::list<js_trajectory_planning_node::NodeIt> Xi_near = planner.FindNodesNear3d(x_a);
    ROS_INFO("Xi_near.size() = %zd", Xi_near.size());

    x_a = planner.AddNodeToTree(x_a, x_closest, Xi_near);
    ROS_INFO("T.size() = %zd", planner.T.size());

    planner.IndentedRosInfo(planner.T);

    ////// x_b
    ROS_INFO("------------------------------------------------------------------");
    ROS_INFO("T.size() = %zd", planner.T.size());
    ROS_INFO("Adding 1 node");
    js_trajectory_planning_node::NodeIt x_b = planner.T.insert(planner.T.begin(),js_trajectory_planning_node::NodeData(js_trajectory_planning_node::Vector3d(2.5, 0, 0)));
    ROS_INFO("T.size() = %zd", planner.T.size());

    ROS_INFO(std::string("Sampled: " + planner.ToString(x_b)).c_str());
    x_closest = planner.GetClosestNodeInTree(x_b);
    ROS_INFO(std::string("Closest: " + planner.ToString(x_closest)).c_str());

    Xi_near = planner.FindNodesNear3d(x_b);
    ROS_INFO("Xi_near.size() = %zd", Xi_near.size());
    planner.NodeListRosInfo(Xi_near);

    x_b = planner.AddNodeToTree(x_b, x_closest, Xi_near);
    ROS_INFO("T.size() = %zd", planner.T.size());

    planner.IndentedRosInfo(planner.T);








    ros::Rate node_rate(2.0f); // 2.0 hz
    while (ros::ok())
    {
        ros::spinOnce();
        node_rate.sleep();
    }
    */
    
    ros::Rate node_rate(2.0f); // 2.0 hz
    while (ros::ok())
    {
        ROS_INFO("Update step");

        tree<js_trajectory_planning_node::NodeData> x_0;
        tree<js_trajectory_planning_node::NodeData> x_goal;
        tree<js_trajectory_planning_node::NodeData> x_agent;
        planner.UpdateX0(x_0.begin());
        planner.UpdateXGoal(x_goal.begin());
        planner.UpdateXAgent(x_agent.begin());
        planner.UpdateXiFree();
        planner.UpdateXiObs();
        
        ROS_INFO("Planning step");
        planner.PerformPlanningCycleOnce();

        js_messages::Trajectory3D trajectory3d_message;
        trajectory3d_publisher.publish(trajectory3d_message);
        ros::spinOnce();
        node_rate.sleep();
    }
    return 0;
}
