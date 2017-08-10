///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief AirSimToRos Implementation
///

#include "airsimros_airsim_to_ros/airsim_to_ros.h"

namespace airsimros
{

AirSimToRos::AirSimToRos()
{
    status_ = 0;
}

uint8_t AirSimToRos::GetStatus()
{
    return status_;
}

void SetStaus(const std::uint8_t& status)
{
    status_ = status;
}
}
