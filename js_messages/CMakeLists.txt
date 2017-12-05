cmake_minimum_required(VERSION 2.8.3)
project(js_messages)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
    Trajectory3D.msg
    Trajectory3DPointStamped.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
