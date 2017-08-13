cmake_minimum_required(VERSION 2.8.3)
project(js_airsim_to_ros_library)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES airsim_to_ros_library
    CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_library(airsim_to_ros_library
    src/airsim_to_ros_class.cpp
)

target_link_libraries(airsim_to_ros_library
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS airsim_to_ros_library
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(airsim_to_ros_library_tests test/test_airsim_to_ros_class.cpp)
    target_link_libraries(airsim_to_ros_library_tests airsim_to_ros_library ${catkin_LIBRARIES})
endif()
