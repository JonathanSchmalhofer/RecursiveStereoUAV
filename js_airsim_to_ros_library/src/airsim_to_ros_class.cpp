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
{
    status_ = 0;
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
    return true;
}
}  // namespace js_airsim_to_ros_library

