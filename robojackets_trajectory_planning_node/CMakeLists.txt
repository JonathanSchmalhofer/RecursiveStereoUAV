cmake_minimum_required(VERSION 2.8.3)
project(robojackets_trajectory_planning_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

# note: the -fPIC flag is for building with Qt...
#       it complains about position-independent code otherwise
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    js_messages
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

# includes
# flann library for KD-Tree data structure
find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

find_package(js_messages REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES trajectory_planning_node ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS roscpp js_messages
    DEPENDS 
)

###########
## Build ##
###########

include_directories(
    include
    ${js_messages_INCLUDE_DIRS}/includes
    ${js_messages_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    "src"
    "src/2dplane"
)

#qt5_add_resources(RESOURCES src/rrt-viewer/main.qrc)

add_executable(rrt_node
    "src/2dplane/2dplane.cpp"
    "src/2dplane/GridStateSpace.cpp"
    "src/2dplane/ObstacleGrid.cpp"
    "src/rrt_node.cpp"
    #${ui_SRC} ${RESOURCES}
)

#qt5_use_modules("rrt_node" Widgets Qml Quick)

target_link_libraries(rrt_node
    ${catkin_LIBRARIES}
)

add_dependencies(rrt_node
    js_messages_generate_messages_cpp
)

#############
## Install ##
#############

install(TARGETS rrt_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############


