cmake_minimum_required(VERSION 2.8.3)
project(flatbuffers_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(flatbuffers_src
    URL https://github.com/google/flatbuffers/archive/v1.7.1.tar.gz
    CONFIGURE_COMMAND cd ../flatbuffers_src && cmake -G Unix\ Makefiles -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND cd ../flatbuffers_src && make -j8
    INSTALL_COMMAND cd ../flatbuffers_src && make install -j8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} flatbuffers_src)
#target_link_libraries(${PROJECT_NAME}
#    ${CATKIN_DEVEL_PREFIX}/lib/libflatbuffers.so)

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
                    CFG_EXTRAS flatbuffers-extras.cmake)
