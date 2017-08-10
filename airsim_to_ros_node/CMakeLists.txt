cmake_minimum_required(VERSION 2.8.3)
project(airsim_to_ros_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(airsim_to_ros_core REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS airsim_to_ros_core
)

###########
## Build ##
###########

include_directories(${airsim_to_ros_core_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/airsim_to_ros_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${airsim_to_ros_core_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  #find_package(rostest REQUIRED)
  #add_rostest_gtest(${PROJECT_NAME}_tests test/airsim_to_ros_node_tests.test test/airsim_to_ros_node_tests.cpp)
  #target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} ${airsim_to_ros_core_LIBRARIES})
endif()
