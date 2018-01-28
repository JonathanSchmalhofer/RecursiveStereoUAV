cmake_minimum_required(VERSION 2.8.3)
project(js_wxwidget_node)

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

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${EIGEN3_INCLUDE_DIRS} ${wxWidgets_LIBRARIES}
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
)

add_executable(wxwidget_node
    src/wxwidget_node.cpp
)

target_link_libraries(wxwidget_node
    ${catkin_LIBRARIES}
    ${wxWidgets_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS wxwidget_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############


