cmake_minimum_required(VERSION 2.8.3)
project(sr_sample_cpp_library)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sample_cpp_library
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(include)

add_library(sample_cpp_library
  src/sample_cpp_class.cpp
)

target_link_libraries(sample_cpp_library
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS sample_cpp_library
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
  catkin_add_gtest(sample_cpp_library_tests test/test_sample_cpp_class.cpp)
  target_link_libraries(sample_cpp_library_tests sample_cpp_library ${catkin_LIBRARIES})
endif()
