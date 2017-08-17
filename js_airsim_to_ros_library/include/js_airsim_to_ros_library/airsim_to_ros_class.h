///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Class to receive Data from AirSim and make available in ROS as ROS-Message
///

#ifndef JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
#define JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_

#include <cstdint>
#include "zeromq_cpp/zmq.hpp"
#include "StarBuffers_generated.h"

struct Star {
    double radius ;
    double mass;
    double volume;
};

namespace js_airsim_to_ros_library
{

class AirSimToRosClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    AirSimToRosClass();

    /// @brief Sets the status of the AirSimToRos object.
    ///
    /// @param status The status of the AirSimToRos node with 0 = ok, else = error code
    void SetStatus(const std::uint8_t& status);

	/// @brief Gets the status of the AirSimToRos object.
    std::uint8_t GetStatus();

  private:
    /// @brief A status indicator.
    std::uint8_t status_;
};
}  // namespace js_airsim_to_ros_library

#endif  // JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
