cmake_minimum_required(VERSION 2.8.3)
project(js_trajectory_planning_node)

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

find_package(dynamicedt3d_catkin REQUIRED)

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

if(wxWidgets_VERSION LESS 3.1)
    message(FATAL_ERROR "wxWidgets is not a high enough version!")
endif()

# OpenGL and GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
    LIBRARIES ${EIGEN3_INCLUDE_DIRS} ${wxWidgets_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${dynamicedt3d_catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
    CATKIN_DEPENDS roscpp js_messages octomap_ros dynamicedt3d_catkin
    DEPENDS octomap
)

###########
## Build ##
###########

include_directories(
    include
    include/js_trajectory_planning_node
    ${js_messages_INCLUDE_DIRS}/includes
    ${js_messages_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${dynamicedt3d_catkin_INCLUDE_DIRS}
    "src"
    "src/3dspace"
)

add_library(birrt_lib
    "src/3dspace/3dspace.cpp"
    "src/3dspace/grid_state_space.cpp"
    "src/3dspace/obstacle_grid.cpp"
    "src/planner_wrapper.cpp"
)

add_executable(trajectory_planning_node
    "src/wxapplicationnode.cpp"
    "src/rrtglcontext.cpp"
    "src/rrtframe.cpp"
    "src/helpers.cpp"
    "src/rrt_node.cpp"
)

target_link_libraries(birrt_lib
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${dynamicedt3d_catkin_LIBRARIES}
)

target_link_libraries(trajectory_planning_node
    ${catkin_LIBRARIES}
    ${wxWidgets_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    birrt_lib
)

add_dependencies(trajectory_planning_node
    js_messages_generate_messages_cpp
)

#############
## Install ##
#############

install(TARGETS birrt_lib
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS trajectory_planning_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".git" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

#    catkin_add_gtest(rrt_3d_node_tree_tests
#        test/tree_test.cpp
#    )
#    catkin_add_gtest(rrt_3d_node_birrt_tests
#        test/birrt_test.cpp
#    )
#    catkin_add_gtest(rrt_3d_node_grid_state_space_tests
#        test/grid_state_space_test.cpp
#    )
    catkin_add_gtest(rrt_3d_node_space_state_space_tests
        test/space_state_space_test.cpp
    )
#    catkin_add_gtest(rrt_3d_node_obstacle_grid_tests
#        test/obstacle_grid_test.cpp
#    )

#    target_link_libraries(rrt_3d_node_tree_tests
#        ${catkin_LIBRARIES}
#        ${wxWidgets_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${GLUT_LIBRARY}
#        ${dynamicedt3d_catkin_LIBRARIES}
#        birrt_lib
#    )

#    target_link_libraries(rrt_3d_node_birrt_tests
#        ${catkin_LIBRARIES}
#        ${wxWidgets_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${GLUT_LIBRARY}
#        ${dynamicedt3d_catkin_LIBRARIES}
#        birrt_lib
#    )

#    target_link_libraries(rrt_3d_node_grid_state_space_tests
#        ${catkin_LIBRARIES}
#        ${wxWidgets_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${GLUT_LIBRARY}
#        ${dynamicedt3d_catkin_LIBRARIES}
#        birrt_lib
#    )

    target_link_libraries(rrt_3d_node_space_state_space_tests
        ${catkin_LIBRARIES}
        ${wxWidgets_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${dynamicedt3d_catkin_LIBRARIES}
        birrt_lib
    )

#    target_link_libraries(rrt_3d_node_obstacle_grid_tests
#        ${catkin_LIBRARIES}
#        ${wxWidgets_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${GLUT_LIBRARY}
#        ${dynamicedt3d_catkin_LIBRARIES}
#        birrt_lib
#    )
endif()
