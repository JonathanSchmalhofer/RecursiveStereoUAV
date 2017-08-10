cmake_minimum_required(VERSION 2.8.3)
project(airsim_to_ros_core)

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

add_library(${PROJECT_NAME}
  src/airsim_to_ros.cpp
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_tests test/airsim_to_ros_test.cpp)
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()