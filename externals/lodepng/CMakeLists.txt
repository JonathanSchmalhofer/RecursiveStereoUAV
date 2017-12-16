cmake_minimum_required(VERSION 2.8.3)
project(lodepng)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lodepng
    CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(lodepng
    src/lodepng.cpp
    include/lodepng.h
)

#############
## Install ##
#############

install(TARGETS lodepng
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############