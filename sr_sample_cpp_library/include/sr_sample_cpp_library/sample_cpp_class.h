/**
* @file   sample_cpp_class.h
* @author Andriy Petlovanyy <software@shadowrobot.com>
*
* Copyright 2015 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* @brief Sample C++ Library.
*
*/

#ifndef SR_SAMPLE_CPP_LIBRARY_SAMPLE_CPP_CLASS_H
#define SR_SAMPLE_CPP_LIBRARY_SAMPLE_CPP_CLASS_H

#include <string>

namespace sr_sample_cpp_library
{

class SampleCppClass
{
public:
    std::string getParameterValueFromName(const std::string &parameter_name) const;
};

}  // namespace sr_sample_cpp_library

#endif  // SR_SAMPLE_CPP_LIBRARY_SAMPLE_CPP_CLASS_H
