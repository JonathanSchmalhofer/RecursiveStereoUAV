cmake_minimum_required(VERSION 2.8.3)
project(dynamicedt3d_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(dynamicedt3d_src
    URL https://github.com/OctoMap/octomap/archive/v1.9.0.tar.gz
    CONFIGURE_COMMAND cd ../dynamicedt3d_src/dynamicEDT3D && cmake . -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND cd ../dynamicedt3d_src/dynamicEDT3D && make -j8
    INSTALL_COMMAND cd ../dynamicedt3d_src/dynamicEDT3D && make install -j8
)

cs_add_library(${PROJECT_NAME}
    src/dependency_tracker.cc
)
add_dependencies(${PROJECT_NAME}
    dynamicedt3d_src
)

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
                    CFG_EXTRAS dynamicedt3d-extras.cmake)
