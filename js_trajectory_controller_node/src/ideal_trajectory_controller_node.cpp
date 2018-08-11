///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief Node to control the trajectory from trajectory planner for an ideal dynamics model.
///
#include <string>
#include <ros/ros.h>
#include <js_common/common.h>
#include <js_trajectory_controller_node/trajectory_point_to_airsim_class.h>
#include <js_messages/Trajectory3D.h>
#include <js_messages/Trajectory3DPointStampedRoot_generated.h>

void Trajectory3dCallback(const js_messages::Trajectory3D::ConstPtr& trajectory3d_message)
{
    ROS_INFO("The sequence is: [%d]", trajectory3d_message->header.seq);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ideal_trajectory_controller_node");
    ros::NodeHandle node_handle;
    
    // Get Parameters from ROS Parameter Server
    std::string port_trajectory_control_to_airsim;
    js_common::TryGetParameter("/recursivestereo/parameters/port_trajectory_control_to_airsim", port_trajectory_control_to_airsim, "6677");
    
    // Setup Subscriber and Publisher
    js_trajectory_controller_node::Trajectory3DPointToAirSimClass ros_to_airsim("tcp://*:" + port_trajectory_control_to_airsim);
    ros::Subscriber trajectory3d_subscriber = node_handle.subscribe("/trajectory_planning/trajectory3d", 1000, Trajectory3dCallback);
    
    ROS_INFO("Starting trajectory controller node");
    
    double z = 0.0f;
    
    ros::Rate node_rate(10); // 10.0 hz
    while (ros::ok())
    {
        z -= 0.1f;
        ros_to_airsim.SetTrajectory3DPointTimeStampSec(0);
        ros_to_airsim.SetTrajectory3DPointTimeStampNsec(0);
        ros_to_airsim.SetTrajectory3DPointPosePositionX(0);
        ros_to_airsim.SetTrajectory3DPointPosePositionY(0);
        ros_to_airsim.SetTrajectory3DPointPosePositionZ(z);
        ros_to_airsim.SetTrajectory3DPointPoseOrientationX(0);
        ros_to_airsim.SetTrajectory3DPointPoseOrientationY(0);
        ros_to_airsim.SetTrajectory3DPointPoseOrientationZ(0);
        ros_to_airsim.SetTrajectory3DPointPoseOrientationW(1);
        ros_to_airsim.SendTrajectory3DPoint();
        ros::spinOnce();
        node_rate.sleep();
    }
    
    return 0;
}
