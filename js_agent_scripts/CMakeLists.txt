cmake_minimum_required(VERSION 2.8.3)
project(js_agent_scripts)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
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


