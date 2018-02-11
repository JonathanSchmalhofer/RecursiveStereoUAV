cmake_minimum_required(VERSION 2.8.3)
project(robojackets_3d_trajectory_planning_node)

add_compile_options(
    $<$<CXX_COMPILER_ID:GNU>:-std=c++14>
)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    js_messages
    octomap_ros
    octomap_msgs
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

# includes
# flann library for KD-Tree data structure
find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

find_package(js_messages REQUIRED)

find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT)

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

# wxWidgets
find_package(wxWidgets REQUIRED net gl core base)
include(${wxWidgets_USE_FILE})

if(wxWidgets_VERSION LESS 2.9)
    message(FATAL_ERROR "wxWidgets is not a high enough version!")
endif()

# OpenGL and GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}
    LIBRARIES trajectory_planning_node ${EIGEN3_INCLUDE_DIRS} ${wxWidgets_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
    CATKIN_DEPENDS roscpp js_messages octomap_ros
    DEPENDS octomap
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
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    "src"
    "src/3dspace"
)

add_executable(rrt_3d_node
    "src/3dspace/3dspace.cpp"
    "src/3dspace/grid_state_space.cpp"
    "src/3dspace/obstacle_grid.cpp"
    "src/wxapplication.cpp"
    "src/planner_wrapper.cpp"
    "src/rrt_node.cpp"
)

target_link_libraries(rrt_3d_node
    ${catkin_LIBRARIES}
    ${wxWidgets_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${OCTOMAP_LIBRARIES}
)

add_dependencies(rrt_3d_node
    js_messages_generate_messages_cpp
)

#############
## Install ##
#############

install(TARGETS rrt_3d_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############


