///
/// @file
/// @copyright Copyright (C) 2018, Jonathan Bryan Schmalhofer
///
/// @brief Common Functions for all ROS nodes
///
#ifndef JS_COMMON_COMMON_H_
#define JS_COMMON_COMMON_H_

#include <cstdint>

namespace js_common
{
    void TryGetParameter(std::string parameter_name, std::string &parameter_value, std::string default_value)
    {
        if (ros::param::has(parameter_name))
        {
            if (ros::param::get(parameter_name, parameter_value))
            {
                ROS_INFO("Got param %s = %s", parameter_name.c_str(), parameter_value.c_str());
            }
            else
            {
                ROS_ERROR("Failed to get param '%s'", parameter_name.c_str());
            }
        }
        else
        {
            ROS_INFO("Using Default Value param %s = %s", parameter_name.c_str(), default_value.c_str());
            parameter_value = default_value;
        }
    }
}  // namespace js_common

#endif  // JS_COMMON_COMMON_H_
