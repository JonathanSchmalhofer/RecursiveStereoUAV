cmake_minimum_required(VERSION 2.8.3)
project(js_airsim_to_ros_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    cv_bridge
    roslaunch
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

find_package(js_airsim_to_ros_library REQUIRED)

find_package(lodepng REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES airsim_to_ros_node
    CATKIN_DEPENDS js_airsim_to_ros_library roscpp image_transport cv_bridge
)

###########
## Build ##
###########

include_directories(
    ${lodepng_INCLUDE_DIRS}
    ${js_airsim_to_ros_library_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(airsim_to_ros_node
    src/airsim_to_ros_node.cpp
)

target_link_libraries(airsim_to_ros_node
    ${js_airsim_to_ros_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(airsim_to_ros_image_publisher_node
    src/airsim_to_ros_image_publisher_node.cpp
)

target_link_libraries(airsim_to_ros_image_publisher_node
    ${lodepng_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
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

# if(CATKIN_ENABLE_TESTING)
    # find_package(rostest REQUIRED)
    # add_rostest_gtest(test_airsim_to_ros_node test/test_airsim_to_ros_node.test test/test_airsim_to_ros_node.cpp)
    # target_link_libraries(test_airsim_to_ros_node ${catkin_LIBRARIES} ${js_airsim_to_ros_node_LIBRARIES})
    
    # add_executable(flatbuffer_zmq_subscriber_node test/flatbuffer_subscriber_node/flatbuffer_zmq_subscriber_node.cpp)
    # add_executable(flatbuffer_zmq_publisher_node test/flatbuffer_publisher_node/flatbuffer_zmq_publisher_node.cpp)
    # add_executable(flatbuffer_zmq_image_publisher_node test/flatbuffer_image_publisher_node/flatbuffer_zmq_image_publisher_node.cpp)
    
    # include_directories(
        # ${js_airsim_to_ros_library_INCLUDE_DIRS}
        # ${catkin_INCLUDE_DIRS}
    # )
    
    # target_link_libraries(flatbuffer_zmq_subscriber_node
        # ${zeromq_catkin_LIBRARIES}
        # ${flatbuffers_catkin_LIBRARIES}
        # ${catkin_LIBRARIES}
    # )
    
    # target_link_libraries(flatbuffer_zmq_publisher_node
        # ${zeromq_catkin_LIBRARIES}
        # ${flatbuffers_catkin_LIBRARIES}
        # ${catkin_LIBRARIES}
    # )
    
    # target_link_libraries(flatbuffer_zmq_image_publisher_node
        # ${zeromq_catkin_LIBRARIES}
        # ${flatbuffers_catkin_LIBRARIES}
        # ${catkin_LIBRARIES}
    # )
# endif()
