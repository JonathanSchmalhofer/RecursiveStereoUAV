cmake_minimum_required(VERSION 2.8.3)
project(js_trajectory_controller_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
#    FILES
#    Trajectory3D.msg
#    Trajectory3DPointStamped.msg
)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
#)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

#find_package(js_airsim_to_ros_library REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ideal_trajectory_controller_node
    CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

include_directories(
#    ${js_airsim_to_ros_library_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(ideal_trajectory_controller_node src/ideal_trajectory_controller_node.cpp)

target_link_libraries(ideal_trajectory_controller_node
#    ${js_airsim_to_ros_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ideal_trajectory_controller_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############


