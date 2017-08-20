///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief AirSimToRos Implementation
///

#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

namespace js_airsim_to_ros_library
{

AirSimToRosClass::AirSimToRosClass(std::string const& addr)
    : zmq_context_(1)
    , zmq_subscriber_(zmq_context_, ZMQ_SUB)
{
    zmq_subscriber_.setsockopt(ZMQ_IDENTITY, "AirSimToRosSubscriber", 5);
    zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq_subscriber_.setsockopt(ZMQ_RCVTIMEO, 5000);
    zmq_subscriber_.connect(addr);
}

void* AirSimToRosClass::GetReceivedMessageData()
{
    return zmq_receivedMessage_.data();
}

std::size_t AirSimToRosClass::GetReceivedMessageSize()
{
    return zmq_receivedMessage_.size();
}

bool AirSimToRosClass::ReceivedMessage()
{    
    int received_bytes = zmq_subscriber_.recv(&zmq_receivedMessage_);
    
    return received_bytes > 0;
}
}  // namespace js_airsim_to_ros_library

