///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Class to transmit a Trajectory3DPoint to AirSim
///
#ifndef JS_TRAJECTORY_CONTROLLER_NODE_TRAJECTORY_POINT_TO_AIRSIM_CLASS_H_
#define JS_TRAJECTORY_CONTROLLER_NODE_TRAJECTORY_POINT_TO_AIRSIM_CLASS_H_

#include <cstdint>
#include <zeromq_cpp/zmq.hpp>
#include <js_messages/Image_generated.h>
#include <js_messages/Trajectory3DPointStampedRoot_generated.h>

namespace js_trajectory_controller_node
{

class Trajectory3DPointToAirSimClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    Trajectory3DPointToAirSimClass(std::string const& addr);
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~Trajectory3DPointToAirSimClass();

private:    
    /// @brief A ZeroMq context object encapsulating functionality dealing with the initialisation and termination.
    zmq::context_t zmq_context_;
    
    /// @brief A ZeroMq socket for publishing messages.
    zmq::socket_t zmq_publisher_;

    /// @brief Holds the attribute Trajectory3DPointStamped.timestamp.sec of the last received message.
    double trajectory3dpoint_timestamp_sec_;

    /// @brief Holds the attribute Trajectory3DPointStamped.timestamp.nsec of the last received message.
    double trajectory3dpoint_timestamp_nsec_;

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
}  // namespace js_trajectory_controller_node

#endif  // JS_TRAJECTORY_CONTROLLER_NODE_TRAJECTORY_POINT_TO_AIRSIM_CLASS_H_
