#ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
#define PROCESSSYNCHRONIZATIONQUEUE_HPP_

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

//Alias an STL compatible allocator of ints that allocates ints from the managed
//shared memory segment.  This allocator will allow to place containers
//in managed shared memory segments
typedef boost::interprocess::allocator<msr::airlib::VehicleCameraBase::ImageResponse, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;

//Alias a vector that uses the previous STL-like allocator
typedef boost::interprocess::vector<msr::airlib::VehicleCameraBase::ImageResponse, ShmemAllocator> ImageResponseVector;

struct ProcessSynchronizationQueue
{
    ProcessSynchronizationQueue()
        : message_available_(false)
    {}
    
    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex      mutex_;
    
    //Condition to wait when the queue is empty
    boost::interprocess::interprocess_condition  condition_empty_;
    
    //Condition to wait when the queue is full
    boost::interprocess::interprocess_condition  condition_full_;
    
    ImageResponseVector response_vector_;
    
    //Is there any message
    bool message_available_;
};

#endif // #ifndef PROCESSSYNCHRONIZATIONQUEUE_HPP_
