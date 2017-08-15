cmake_minimum_required(VERSION 2.8.3)
project(js_airsim_to_ros_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(js_airsim_to_ros_library REQUIRED)

find_package(zeromq_catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS js_airsim_to_ros_library zeromq_catkin
)

###########
## Build ##
###########

include_directories(
  ${js_airsim_to_ros_library_INCLUDE_DIRS}
  ${zeromq_catkin_INCLUDE_DIRS}
)

add_executable(airsim_to_ros_node src/airsim_to_ros_node.cpp)

target_link_libraries(airsim_to_ros_node
  ${js_airsim_to_ros_library_LIBRARIES}
  ${zeromq_catkin_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS airsim_to_ros_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_airsim_to_ros_node test/test_airsim_to_ros_node.test test/test_airsim_to_ros_node.cpp)
  target_link_libraries(test_airsim_to_ros_node ${catkin_LIBRARIES} ${js_airsim_to_ros_node_LIBRARIES} ${zeromq_catkin_LIBRARY})
endif()
