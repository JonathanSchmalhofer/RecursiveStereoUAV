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
#include <std_msgs/Bool.h>
#include <js_messages/Trajectory3D.h>
#include <js_messages/Trajectory3DPointStampedRoot_generated.h>

class TrajectoryControllerNode
{
    public:
        TrajectoryControllerNode()
        {
            // Get Parameters from ROS Parameter Server
            std::string port_trajectory_control_to_airsim;
            js_common::TryGetParameter("/recursivestereo/parameters/port_trajectory_control_to_airsim", port_trajectory_control_to_airsim, "6677");
            
            // Setup Subscriber and ZeroMQ Publisher
            subscriber_           = node_handle_.subscribe("/trajectory_planning/trajectory3d", 1, &TrajectoryControllerNode::Trajectory3dCallback, this);
            subscriber_heartbeat_ = node_handle_.subscribe("/airsim/heartbeat",                 1, &TrajectoryControllerNode::ResendCallback,       this);
            ros_to_airsim_        = new js_trajectory_controller_node::Trajectory3DPointToAirSimClass("tcp://*:" + port_trajectory_control_to_airsim);
            
            // Temporary
            current_z_ = 0;
        }
    
        ~TrajectoryControllerNode()
        {
            delete ros_to_airsim_;
            ros_to_airsim_ = NULL;
        }

        void Trajectory3dCallback(const js_messages::Trajectory3D::ConstPtr& trajectory3d_message)
        {
            ROS_INFO("The sequence is: [%d]", trajectory3d_message->header.seq);

            current_z_ -= 0.1f;
            ros_to_airsim_->SetTrajectory3DPointTimeStampSec(0);
            ros_to_airsim_->SetTrajectory3DPointTimeStampNsec(0);
            ros_to_airsim_->SetTrajectory3DPointPosePositionX(0);
            ros_to_airsim_->SetTrajectory3DPointPosePositionY(0);
            ros_to_airsim_->SetTrajectory3DPointPosePositionZ(current_z_);
            ros_to_airsim_->SetTrajectory3DPointPoseOrientationX(0);
            ros_to_airsim_->SetTrajectory3DPointPoseOrientationY(0);
            ros_to_airsim_->SetTrajectory3DPointPoseOrientationZ(0);
            ros_to_airsim_->SetTrajectory3DPointPoseOrientationW(1);
            ros_to_airsim_->SendTrajectory3DPoint();
        }

        void ResendCallback(const std_msgs::Bool::ConstPtr& heartbeat_message)
        {
			ROS_INFO("Received a heartbeat - sending last message again");
			// Send last set message again
            ros_to_airsim_->SendTrajectory3DPoint();
        }

    private:
        ros::NodeHandle                                                node_handle_; 
        js_trajectory_controller_node::Trajectory3DPointToAirSimClass* ros_to_airsim_;
        ros::Subscriber                                                subscriber_;
        ros::Subscriber                                                subscriber_heartbeat_;
        double                                                         current_z_;

};//End of class TrajectoryControllerNode

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_controller_node");
        
    ROS_INFO("Starting trajectory controller node");
    
    // Create an object of class TrajectoryControllerNode that will take care of everything
    TrajectoryControllerNode trajectory_controller_node_;

    // Let's go
    ros::spin();
    
    return 0;
}
