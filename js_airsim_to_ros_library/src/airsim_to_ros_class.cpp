///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief AirSimToRos Implementation
///

#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

namespace js_airsim_to_ros_library
{

AirSimToRosClass::AirSimToRosClass()
    : zmq_context_(1)
    , zmq_subscriber_(zmq_context_, ZMQ_SUB)
{
    status_         = 0;
    zmq_subscriber_.setsockopt(ZMQ_IDENTITY, "AirSimToRosSubscriber", 5);
    zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq_subscriber_.connect("tcp://localhost:5565");
}

std::uint8_t AirSimToRosClass::GetStatus()
{
    return status_;
}

void AirSimToRosClass::SetStatus(const std::uint8_t& status)
{
    status_ = status;
}

bool AirSimToRosClass::ReceivedMessage()
{
    zmq_subscriber_.recv(&zmq_receivedMessage_);
    
    return true;
}
}  // namespace js_airsim_to_ros_library

