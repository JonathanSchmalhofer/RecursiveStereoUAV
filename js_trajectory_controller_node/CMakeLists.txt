cmake_minimum_required(VERSION 2.8.3)
project(js_trajectory_controller_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    js_messages
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

find_package(js_messages REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ideal_trajectory_controller_node
    CATKIN_DEPENDS roscpp js_messages
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(ideal_trajectory_controller_node
    src/ideal_trajectory_controller_node.cpp)

target_link_libraries(ideal_trajectory_controller_node
    ${catkin_LIBRARIES}
)

add_dependencies(ideal_trajectory_controller_node
    js_messages_generate_messages_cpp
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


