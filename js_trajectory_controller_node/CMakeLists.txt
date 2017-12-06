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

find_package(zeromq_catkin REQUIRED)

find_package(flatbuffers_catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ideal_trajectory_controller_node
    CATKIN_DEPENDS roscpp js_messages flatbuffers_catkin zeromq_catkin
)

###########
## Build ##
###########

include_directories(
    include
    ${zeromq_catkin_INCLUDE_DIRS}
    ${flatbuffers_catkin_INCLUDE_DIRS}
    ${js_messages_INCLUDE_DIRS}/includes
    ${js_messages_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(ideal_trajectory_controller_node
    src/ideal_trajectory_controller_node.cpp
    src/trajectory_point_to_airsim_class.cpp
)

target_link_libraries(ideal_trajectory_controller_node
    ${zeromq_catkin_LIBRARIES}
    ${flatbuffers_catkin_LIBRARIES}
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


