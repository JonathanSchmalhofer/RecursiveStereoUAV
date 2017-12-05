///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Class to receive Data from ROS and make them available in AirSim
///
#ifndef ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_
#define ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_

#include <cstdint>
#include <zeromq_cpp/zmq.hpp>
#include <js_messages/Trajectory3DPointStampedRoot_generated.h>

namespace ros_to_airsim
{

class RosToAirSimClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RosToAirSimClass(std::string const& addr);
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RosToAirSimClass();
    
    // @brief Returns true if a new, full message was received via ZeroMq, false otherwise
    int8_t ReceivedMessage();
    
    // @brief Returns the attribute Trajectory3DPointStamped.time.sec
    uint32_t GetTrajectory3DPointTimeSec();
    
    // @brief Returns the attribute Trajectory3DPointStamped.time.nsec
    uint32_t GetTrajectory3DPointTimeNsec();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.position.x
    double GetTrajectory3DPointPosePositionX();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.position.y
    double GetTrajectory3DPointPosePositionY();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.position.z
    double GetTrajectory3DPointPosePositionZ();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.orientation.x
    double GetTrajectory3DPointPoseOrientationX();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.orientation.y
    double GetTrajectory3DPointPoseOrientationY();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.orientation.z
    double GetTrajectory3DPointPoseOrientationZ();
    
    // @brief Returns the attribute Trajectory3DPointStamped.pose.orientation.w
    double GetTrajectory3DPointPoseOrientationW();

private:    
    /// @brief A ZeroMq context object encapsulating functionality dealing with the initialisation and termination.
    zmq::context_t zmq_context_;
    
    /// @brief A ZeroMq socket for subscribing to incoming messages.
    zmq::socket_t zmq_subscriber_;

    /// @brief Holds the attribute Trajectory3DPointStamped.time.sec of the last received message.
    uint32_t trajectory3dpoint_time_sec_;

    /// @brief Holds the attribute Trajectory3DPointStamped.time.nsec of the last received message.
    uint32_t trajectory3dpoint_time_nsec_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.position.x of the last received message.
    double trajectory3dpoint_pose_position_x_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.position.y of the last received message.
    double trajectory3dpoint_pose_position_y_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.position.z of the last received message.
    double trajectory3dpoint_pose_position_z_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.orientation.x of the last received message.
    double trajectory3dpoint_pose_orientation_x_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.orientation.y of the last received message.
    double trajectory3dpoint_pose_orientation_y_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.orientation.z of the last received message.
    double trajectory3dpoint_pose_orientation_z_;
    
    /// @brief Holds the attribute Trajectory3DPointStamped.pose.orientation.w of the last received message.
    double trajectory3dpoint_pose_orientation_w_;
};
}  // namespace ros_to_airsim

#endif  // ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_
