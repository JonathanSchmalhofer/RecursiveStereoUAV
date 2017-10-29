#include "StereoImageRpcClient.hpp" // only for the AirSim includes needed
#include "ProcessSynchronizationQueue.hpp"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <string>
#include <cstdlib> //std::system

using namespace boost::interprocess;

//Alias an STL compatible allocator of ints that allocates ints from the managed
//shared memory segment.  This allocator will allow to place containers
//in managed shared memory segments
typedef allocator<msr::airlib::VehicleCameraBase::ImageResponse, managed_shared_memory::segment_manager> ShmemAllocator;

//Alias a vector that uses the previous STL-like allocator
typedef vector<msr::airlib::VehicleCameraBase::ImageResponse, ShmemAllocator> ImageResponseVector;

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <cstring>

int main(int argc, const char *argv[])
{
    std::cout << "Press ENTER to start ZeroMQ bridge application" << std::endl; std::cin.get();
    //Create a shared memory object.
    shared_memory_object shm
       (open_only                    //only create
       ,"shared_memory_mutex"              //name
       ,read_write                   //read-write mode
       );
    
    try{
        //Map the whole shared memory in this process
        mapped_region region
           (shm                       //What to map
           ,read_write //Map it as read-write
           );
        
        //Get the address of the mapped region
        void * addr       = region.get_address();
         
        //Obtain a pointer to the shared structure
        ProcessSynchronizationQueue * data = static_cast<ProcessSynchronizationQueue*>(addr);
        
        //Print messages until the other process marks the end
        bool end_loop = false;
        do
        {
            scoped_lock<interprocess_mutex> lock(data->mutex_);
            if(!data->message_available_)
            {
                data->condition_empty_.wait(lock);
            }
            else
            {
                // PROCESSING COMES HERE
                // TODO
                
                //Open the managed segment
                managed_shared_memory segment(open_only, "shared_memory_stereo_images_and_depth_image");
                
                //Find the vector using the c-string name
                ImageResponseVector *stereo_image_depth_image_vector = segment.find<ImageResponseVector>("ImageResponseVector").first;
                
                /*
                 * TODO: Do we need this?
                //When done, destroy the vector from the segment
                segment.destroy<ImageResponseVector>("ImageResponseVector");
                */
                
                std::cout << "Number of images is " << unsigned(stereo_image_depth_image_vector->size()) << std::endl;
                
                //Notify the other process that the buffer is empty
                data->message_available_ = false;
                data->condition_full_.notify_one();
            }
        }
        while(!end_loop);
    }
    catch(interprocess_exception &ex)
    {
        shared_memory_object::remove("shared_memory_mutex");
        std::cout << ex.what() << std::endl;
        return 1;
    }
    
    //Erase shared memory
    shared_memory_object::remove("shared_memory_mutex");
    return 0;
}
