/**
* @file   test_sample_cpp_class.cpp
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
* @brief Test for sample C++ class.
*
*/

#include <gtest/gtest.h>
#include <string>

#include "sr_sample_cpp_library/sample_cpp_class.h"

using sr_sample_cpp_library::SampleCppClass;

TEST(SampleCppClassTestSuite, checkGetParameterValueFromName)
{
    SampleCppClass sample_object;

    std::string param1("my_cpp_param_one");
    std::string param2("my_cpp_param_two");

    std::string expected_result1("my_cpp_param_one_test");
    std::string expected_result2("my_cpp_param_two_test");

    ASSERT_EQ(expected_result1, sample_object.getParameterValueFromName(param1));
    ASSERT_EQ(expected_result2, sample_object.getParameterValueFromName(param2));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
