cmake_minimum_required(VERSION 2.8.3)
project(octomap_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(octomap_src
  URL https://github.com/OctoMap/octomap/archive/v1.9.0.tar.gz
  CONFIGURE_COMMAND cd ../octomap_src && cmake . -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND cd ../octomap_src && make -j8
  INSTALL_COMMAND cd ../octomap_src && make install -j8 #mkdir -p ${CATKIN_DEVEL_PREFIX}/include/zeromq_cpp && cp ../zeromq_cpp/zmq.hpp
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} octomap_src)

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          CFG_EXTRAS octomap-extras.cmake)
