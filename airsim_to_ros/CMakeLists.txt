cmake_minimum_required(VERSION 2.8.3)
project(airsim_to_ros)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

set(catkin_package_DEPENDENCIES
  roscpp
)

## Find catkin macros and libraries
find_package(catkin REQUIRED ${catkin_package_DEPENDENCIES})

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  include/airsimros_airsim_to_ros/airsim_to_ros.h
  src/airsim_to_ros.cpp
  src/airsim_to_ros_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/airsim_to_ros.cpp
  src/airsim_to_ros_node.cpp
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}_tests
#  test/airsim_to_ros_test.cpp
#)
#if(TARGET ${PROJECT_NAME}_tests)
#  target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
#endif()