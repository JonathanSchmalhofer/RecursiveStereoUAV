#ifndef AIRSIM_RPC_CLIENT_HPP_
#define AIRSIM_RPC_CLIENT_HPP_

// Synchronization
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <cstdio>
#include "ProcessSynchronizationQueue.hpp"

// Vector
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <string>
#include <cstdlib> //std::system

#include <cstdint> //std::uint8_t



using namespace boost::interprocess;

//Define an STL compatible allocator of ints that allocates from the managed_shared_memory.
//This allocator will allow placing containers in the segment
typedef allocator<std::uint8_t, managed_shared_memory::segment_manager>  SharedMemoryAllocator;

//Alias a vector that uses the previous STL-like allocator so that allocates
//its values from the segment
typedef vector<std::uint8_t, SharedMemoryAllocator> ImageVector;

//Erase previous shared memory and schedule erasure on exit
struct shared_memory_remove_helper
{
    shared_memory_remove_helper() { shared_memory_object::remove("MutexSharedMemory"); shared_memory_object::remove("VectorSharedMemory"); }
    ~shared_memory_remove_helper(){ shared_memory_object::remove("MutexSharedMemory"); shared_memory_object::remove("VectorSharedMemory"); }
} remover_helper;

const unsigned size_full_hd_uint8 = 1920 * 1080 * sizeof(std::uint8_t);


#endif // #ifndef AIRSIM_RPC_CLIENT_HPP_
