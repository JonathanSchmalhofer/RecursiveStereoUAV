#ifndef JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
#define JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_

#include <string>

namespace js_airsim_to_ros_library
{

class AirSimToRosClass
{
public:
    std::string getParameterValueFromName(const std::string &parameter_name) const;
};

}  // namespace js_airsim_to_ros_library

#endif  // JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
