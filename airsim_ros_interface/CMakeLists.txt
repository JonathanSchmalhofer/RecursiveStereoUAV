cmake_minimum_required(VERSION 3.0.0)
project(airsim_ros_interface)

#SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /showIncludes")

################################################################################
#
# Configuration
#
################################################################################

set(AIRSIM_ROOT_DIR "D:/AirSim")


################################################################################


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(ExternalProject)

################################################################################
#
# Download, Compile & install external libraries
#
################################################################################

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.65 REQUIRED COMPONENTS date_time)

################################################################################
#
# Compile & install the library
#
################################################################################

include(TargetArch)

set(AIRSIM_ROS_INTERFACE_VERSION_MAJOR 1)
set(AIRSIM_ROS_INTERFACE_VERSION_MINOR 0)
set(AIRSIM_ROS_INTERFACE_VERSION_PATCH 1)

target_architecture(TARGET_ARCH)

if (${TARGET_ARCH} STREQUAL "i386")
    message(STATUS "Compiling for 32-bit")
    set(AIRSIM_ROS_INTERFACE_ARCH_DEF "AIRSIM_ROS_INTERFACE_ARCH_X86")
    set(AIRSIM_ROS_INTERFACE_TARGET_ARCH "x86")
    set(AIRSIM_ROS_INTERFACE_DEB_ARCH "i386")
elseif(${TARGET_ARCH} STREQUAL "x86_64")
    message(STATUS "Compiling for 64-bit")
    set(AIRSIM_ROS_INTERFACE_ARCH_DEF "AIRSIM_ROS_INTERFACE_ARCH_X64")
    set(AIRSIM_ROS_INTERFACE_TARGET_ARCH "x64")
    set(AIRSIM_ROS_INTERFACE_DEB_ARCH "amd64")
endif()

if (WIN32)
    set(AIRSIM_ROS_INTERFACE_OS_DEF "AIRSIM_ROS_INTERFACE_WIN32")
elseif (LINUX)
    set(AIRSIM_ROS_INTERFACE_OS_DEF "AIRSIM_ROS_INTERFACE_LINUX")
elseif (APPLE)
    set(AIRSIM_ROS_INTERFACE_OS_DEF "AIRSIM_ROS_INTERFACE_MAC")
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)

set(ZMQ_INCLUDES
    build/include/zeromq_src
    build/include
)

set(FLATBUFFERS_INCLUDES
    build/include/flatbuffers_src
)

set(AIRSIM_INCLUDES
    ${AIRSIM_ROOT_DIR}/AirLib/include
	${AIRSIM_ROOT_DIR}/external/rpclib/include
	${AIRSIM_ROOT_DIR}/AirLib/deps/eigen3
)

set(AIRSIM_LIBRARIES_RELEASE
	${AIRSIM_ROOT_DIR}/external/rpclib/build/output/lib/Release/rpc.lib
	${AIRSIM_ROOT_DIR}/AirLib/lib/x64/Release/AirLib.lib
)
set(AIRSIM_LIBRARIES_DEBUG
	${AIRSIM_ROOT_DIR}/external/rpclib/build/output/lib/Debug/rpc.lib
	${AIRSIM_ROOT_DIR}/AirLib/lib/x64/Debug/AirLib.lib
)
set(ZMQ_LIBRARIES_DEBUG
	${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Debug/libzmq.lib
)


include_directories(
    include
    ${AIRSIM_INCLUDES}
    ${ZMQ_INCLUDES}
    ${FLATBUFFERS_INCLUDES}
    ${Boost_INCLUDE_DIRS}
    ../js_messages/include/
)

file(GLOB_RECURSE AIRSIM_ROS_INTERFACE_HEADERS
    include/*.h
    include/*.hpp
)

if(AIRSIM_ROS_INTERFACE_NAME_SUFFIX)
    set(OUTPUT_LIBRARY_NAME ${CMAKE_PROJECT_NAME}-${AIRSIM_ROS_INTERFACE_NAME_SUFFIX})
else()
    set(OUTPUT_LIBRARY_NAME ${CMAKE_PROJECT_NAME})
endif()

add_executable(${OUTPUT_LIBRARY_NAME}
    src/dummy_main.cpp
)

add_executable(airsim_rpc_client
    src/airsim_rpc_client.cpp
    src/StereoImageRpcClient.cpp
    ${AIRSIM_ROS_INTERFACE_HEADERS}
)

add_executable(airsim_zmq_bridge
    src/airsim_zmq_bridge.cpp
    src/StereoImageRpcClient.cpp
    src/ros_to_airsim_class.cpp
    ${AIRSIM_ROS_INTERFACE_HEADERS}
)

target_link_libraries(airsim_rpc_client
    "$<$<CONFIG:Debug>:${AIRSIM_LIBRARIES_DEBUG}>"
    "$<$<CONFIG:Release>:${AIRSIM_LIBRARIES_RELEASE}>"
    ${Boost_LIBRARIES}
)

target_link_libraries(airsim_zmq_bridge
    "$<$<CONFIG:Debug>:${AIRSIM_LIBRARIES_DEBUG}>"
    "$<$<CONFIG:Debug>:${ZMQ_LIBRARIES_DEBUG}>"
    "$<$<CONFIG:Release>:${AIRSIM_LIBRARIES_RELEASE}>"
    ${Boost_LIBRARIES}
)

# When building via conan, respect the compilation settings.
if ("${CONAN_LINK_RUNTIME}" STREQUAL "/MT")
    set(AIRSIM_ROS_INTERFACE_MSVC_STATIC_RUNTIME ON)
endif()

# MSVC static runtime support
#
# While this pollutes global flags (when using add_library), you would not want to
# build with a disparity anyway. (also, CMake still has no support for this, so you
# would end up doing something like this yourself).
if (AIRSIM_ROS_INTERFACE_MSVC_STATIC_RUNTIME)
	# Set compiler options.
	set(variables
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_MINSIZEREL
        CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_RELWITHDEBINFO
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_MINSIZEREL
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_RELWITHDEBINFO
	)

	message(STATUS
		"MSVC -> forcing use of statically-linked runtime."
	)

	foreach(variable ${variables})
		if(${variable} MATCHES "/MD")
            string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
		endif()
	endforeach()

endif()

install(TARGETS ${OUTPUT_LIBRARY_NAME} DESTINATION lib)
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN "*.inl"
    PATTERN "*.in" EXCLUDE
)


################################################################################
#
# CPack
#
################################################################################

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "An AirSim Interface to ROS")
set(CPACK_PACKAGE_DESCRIPTION "An AirSim Interface to ROS")
set(CPACK_PACKAGE_VENDOR "AIRSIM_ROS_INTERFACE")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md")
set(CPACK_PACKAGE_VERSION_MAJOR "${AIRSIM_ROS_INTERFACE_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${AIRSIM_ROS_INTERFACE_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${AIRSIM_ROS_INTERFACE_VERSION_PATCH}")
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_BINARY_DIR}/output/pkg")
set(CPACK_PACKAGE_EXECUTABLES "libairsim_ros_interface" "airsim_ros_interface")
set(CPACK_PACKAGE_NAME "lib${CMAKE_PROJECT_NAME}_${AIRSIM_ROS_INTERFACE_DEB_ARCH}")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${AIRSIM_ROS_INTERFACE_DEB_ARCH})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Jonathan Bryan Schmalhofer")
set(CPACK_DEBIAN_PACKAGE_CONTACT "Jonathan Bryan Schmalhofer")
if(WIN32)
    set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
endif()

if(WIN32)
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "libairsim_ros_interface ${AIRSIM_ROS_INTERFACE_VERSION_MAJOR}.${AIRSIM_ROS_INTERFACE_VERSION_MINOR}.${AIRSIM_ROS_INTERFACE_VERSION_PATCH}")
    set(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} AIRSIM_ROS_INTERFACE")
    #set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\github.com/JonathanSchmalhofer/RecursiveStereoUAV")
    set(CPACK_NSIS_MODIFY_PATH ON)
endif()
include(CPack)
