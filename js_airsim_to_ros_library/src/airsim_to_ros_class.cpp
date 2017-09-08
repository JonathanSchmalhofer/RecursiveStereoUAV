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

    received_message_size_ = 0;
    received_message_data_ = NULL;
}

void* AirSimToRosClass::GetReceivedMessageData()
{
    //return zmq_received_message_.data();
    return (void*)received_message_data_;
}

std::size_t AirSimToRosClass::GetReceivedMessageSize()
{
    //return zmq_received_message_.size();
    return received_message_size_;
}

bool AirSimToRosClass::ReceivedMessage()
{
    zmq::message_t zmq_received_message;
    int received_bytes = zmq_subscriber_.recv(&zmq_received_message);

    if (received_bytes > 0)
    {
      received_message_size_ = zmq_received_message.size();
      received_message_data_ = (std::uint8_t*)realloc(received_message_data_, zmq_received_message.size());
      memcpy((void *)received_message_data_,(void *)zmq_received_message.data(), (size_t)received_message_size_);
    }

    
    return received_bytes > 0;
}
}  // namespace js_airsim_to_ros_library

