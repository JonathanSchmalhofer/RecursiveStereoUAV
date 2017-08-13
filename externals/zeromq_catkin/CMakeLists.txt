cmake_minimum_required(VERSION 2.8.3)
project(zeromq_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(zeromq_src
  URL https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz
  CONFIGURE_COMMAND cd ../zeromq_src && ./configure --with-pic --prefix=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND cd ../zeromq_src && make -j8
  INSTALL_COMMAND cd ../zeromq_src && make install -j8
)

ExternalProject_Add(zeromq_cpp
  GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
  GIT_TAG d9f0f016c07046742738c65e1eb84722ae32d7d4
  DEPENDS zeromq_src
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/include/zeromq_cpp && cp ../zeromq_cpp/zmq.hpp ${CATKIN_DEVEL_PREFIX}/include/zeromq_cpp/zmq.hpp
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} zeromq_src zeromq_cpp)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libzmq${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          CFG_EXTRAS zeromq-extras.cmake)
