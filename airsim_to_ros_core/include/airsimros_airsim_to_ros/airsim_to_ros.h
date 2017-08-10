///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Class to receive Data from AirSim and make available in ROS as ROS-Message
///

#ifndef AIRSIMROS_AIRSIM_TO_ROS_H_
#define AIRSIMROS_AIRSIM_TO_ROS_H_

#include <stdint.h>
#include <ros/ros.h>

namespace airsimros
{

class AirSimToRos
{
  public:
    /// @brief Initializes a AirSimToRos class instance.
    AirSimToRos();

    /// @brief Sets the status of the AirSimToRos object.
    ///
    /// @param status The status of the AirSimToRos node with 0 = ok, else = error code
    void SetStatus(const std::uint8_t& status);

	/// @brief Gets the status of the AirSimToRos object.
    uint8_t GetStatus();

  private:
    /// @brief A status indicator.
    uint8_t status_;
};
}

#endif
