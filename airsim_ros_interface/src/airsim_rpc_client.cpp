#include "StereoImageRpcClient.hpp"
#include "ProcessSynchronizationQueue.hpp"

/*
 * not needed if mutex is used for process synchronization
#include <chrono>  // for sleep_for
#include <thread>  // for sleep_for - see: https://stackoverflow.com/questions/4184468/sleep-for-milliseconds
*/

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

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
#include <cstdio>

int main(int argc, const char *argv[])
{
    std::cout << "Press ENTER to start StereoImageRpcClient" << std::endl; std::cin.get();
    
    StereoImageRpcClient *client = new StereoImageRpcClient();    
    client->connect();
    msr::airlib::Pose initial_pose = client->getPositionAndOrientation();
    
    std::cout << "Initial Pose:" << std::endl;
    std::cout << " x = " << initial_pose.position.x() << std::endl;
    std::cout << " y = " << initial_pose.position.y() << std::endl;
    std::cout << " z = " << initial_pose.position.z() << std::endl; ;
    std::cout << " w = " << initial_pose.orientation.w() << std::endl; ;
    std::cout << " x = " << initial_pose.orientation.x() << std::endl; ;
    std::cout << " y = " << initial_pose.orientation.y() << std::endl; ;
    std::cout << " z = " << initial_pose.orientation.z() << std::endl;
    
    std::cout << "Start Main Loop?" << std::endl; std::cin.get();
    
    msr::airlib::Pose delta_pose, new_pose = initial_pose;
    
    //Shared memory front-end that is able to construct objects
    //associated with a c-string. Erase previous shared memory with the name
    //to be used and create the memory segment at the specified address and initialize resources
    shared_memory_object::remove("shared_memory_stereo_images_and_depth_image");
    managed_shared_memory segment(create_only,
                                  "shared_memory_stereo_images_and_depth_image",  //segment name
                                  65536);  //segment size in bytes
    
    //Initialize shared memory STL-compatible allocator
    const ShmemAllocator alloc_inst (segment.get_segment_manager());
    
    //Construct a shared memory
    ImageResponseVector *stereo_image_depth_image_vector = segment.construct<ImageResponseVector>("ImageResponseVector")(alloc_inst);//first ctor parameter
    
    /////////////////////////////////////////////////////////
    //Erase previous shared memory
    shared_memory_object::remove("shared_memory_mutex");
    
    //Create a shared memory object.
    shared_memory_object shm
       (create_only               //only create
       ,"shared_memory_mutex"     //name
       ,read_write                //read-write mode
       );
    //Set size
    shm.truncate(sizeof(ProcessSynchronizationQueue));
    
    //Map the whole shared memory in this process
    mapped_region process_synchronization_region
      (shm                       //What to map
      ,read_write //Map it as read-write
      );
    
    //Get the address of the mapped region
    void *adress_mapped_region = process_synchronization_region.get_address();
    
    //Construct the shared structure in memory
    ProcessSynchronizationQueue *process_synchronization_data = new (adress_mapped_region) ProcessSynchronizationQueue;
    
    while( true )
    {
        stereo_image_depth_image_vector->clear();
        
        scoped_lock<interprocess_mutex> lock(process_synchronization_data->mutex_);
        
        if(process_synchronization_data->message_available_)
        {
            process_synchronization_data->condition_full_.wait(lock);
        }
        
        delta_pose = msr::airlib::Pose(msr::airlib::Vector3r(-0.01f, 0.0f, 0.01f), msr::airlib::Quaternionr(1.0f, 0.0f, 0.0f, 0.0f));
        new_pose = new_pose - delta_pose;
        
        client->setPositionAndOrientation(new_pose);
        
        std::cout << "Collided? Answer: " << client->hasCollided() << std::endl;
        
        std::vector<msr::airlib::VehicleCameraBase::ImageResponse> received_image_response_vector = client->requestStereoImagesAndDepthImage();
        
        //Insert data in the vector
        for (unsigned i=0; i<received_image_response_vector.size(); i++)
        {
            stereo_image_depth_image_vector->push_back(received_image_response_vector.at(i));
        }
        
        std::cout << "Number of images is " << stereo_image_depth_image_vector->size() << std::endl;
        
        //Notify to the other process that there is a message
        process_synchronization_data->condition_empty_.notify_one();
        
        //Mark message buffer as full
        process_synchronization_data->message_available_ = true;
        /*
         * not needed if mutex is used for process synchronization
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        */
    }
    
    delete client;
    
    shared_memory_object::remove("sharedMemoryStereoImagesAndDepthImage");
    shared_memory_object::remove("shared_memory_mutex");
    
    return 0;
}
