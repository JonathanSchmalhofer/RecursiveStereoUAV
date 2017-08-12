#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

#include <string>

namespace js_airsim_to_ros_library
{

std::string AirSimToRosClass::getParameterValueFromName(const std::string &parameter_name) const
{
    std::string result = parameter_name;
    result.append("_test");
    return result;
}

}  // namespace js_airsim_to_ros_library
