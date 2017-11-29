///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Definition of additional datatypes for shared memory queue adapted from boost example and collected includes
///
#ifndef AIRSIM_ZMQ_BRIDGE_HPP_
#define AIRSIM_ZMQ_BRIDGE_HPP_

#include <js_airsim_to_ros_library/Image_generated.h>
#include <zeromq_cpp/zmq.hpp>

// Synchronization
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <cstring>
#include "ProcessSynchronizationQueue.hpp"

// Vector
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <string>
#include <cstdlib> //std::system

using namespace boost::interprocess;

//Define an STL compatible allocator of ints that allocates from the managed_shared_memory.
//This allocator will allow placing containers in the segment
typedef allocator<std::uint8_t, managed_shared_memory::segment_manager>  SharedMemoryAllocator;

//Alias a vector that uses the previous STL-like allocator so that allocates
//its values from the segment
typedef vector<std::uint8_t, SharedMemoryAllocator> ImageVector;


#endif // #ifndef AIRSIM_ZMQ_BRIDGE_HPP_
