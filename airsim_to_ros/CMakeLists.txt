cmake_minimum_required(VERSION 2.8.3)
project(airsim_to_ros)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

set(catkin_package_DEPENDENCIES
  roscpp
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  ${catkin_package_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  include/airsimros_airsim_to_ros/airsim_to_ros.h
  src/airsim_to_ros.cpp
  src/airsim_to_ros_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/airsim_to_ros.cpp
  src/airsim_to_ros_node.cpp
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_lib
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_tests test/airsim_to_ros_test.test test/airsim_to_ros_test.cpp)
  target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
endif()

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}_tests
#  test/airsim_to_ros_test.cpp
#)
#if(TARGET ${PROJECT_NAME}_tests)
#  target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
#endif()