cmake_minimum_required(VERSION 2.8.3)
project(sr_sample_cpp_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(sr_sample_cpp_library REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS sr_sample_cpp_library
)

###########
## Build ##
###########

include_directories(${sr_sample_cpp_library_INCLUDE_DIRS})

add_executable(sample_cpp_node src/sample_cpp_node.cpp)

target_link_libraries(sample_cpp_node
  ${sr_sample_cpp_library_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS sample_cpp_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_sample_cpp_node test/test_sample_cpp_node.test test/test_sample_cpp_node.cpp)
  target_link_libraries(test_sample_cpp_node ${catkin_LIBRARIES} ${sr_sample_cpp_library_LIBRARIES})
endif()
