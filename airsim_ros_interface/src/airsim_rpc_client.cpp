#include "StereoImageRpcClient.hpp"

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



int main(int argc, const char *argv[])
{
    //std::cout << "Press ENTER to start StereoImageRpcClient" << std::endl; std::cin.get();

    StereoImageRpcClient *client = new StereoImageRpcClient();    
    client->Connect();
    msr::airlib::Pose initial_pose = client->GetPositionAndOrientation();

    std::cout << "Initial Pose:" << std::endl;
    std::cout << " x = " << initial_pose.position.x() << std::endl;
    std::cout << " y = " << initial_pose.position.y() << std::endl;
    std::cout << " z = " << initial_pose.position.z() << std::endl; ;
    std::cout << " w = " << initial_pose.orientation.w() << std::endl; ;
    std::cout << " x = " << initial_pose.orientation.x() << std::endl; ;
    std::cout << " y = " << initial_pose.orientation.y() << std::endl; ;
    std::cout << " z = " << initial_pose.orientation.z() << std::endl;

    //std::cout << "Start Main Loop?" << std::endl; std::cin.get();

    msr::airlib::Pose delta_pose, new_pose = initial_pose;

    //Erase previous shared memory and schedule erasure on exit
    struct shared_memory_remove_helper
    {
        shared_memory_remove_helper() { shared_memory_object::remove("MutexSharedMemory"); shared_memory_object::remove("VectorSharedMemory"); }
        ~shared_memory_remove_helper(){ shared_memory_object::remove("MutexSharedMemory"); shared_memory_object::remove("VectorSharedMemory"); }
    } remover_helper;

    //Create a shared memory object.
    shared_memory_object mutex_shared_memory(create_only, "MutexSharedMemory", read_write);

    //Create a new segment with given name and size
    const unsigned size_full_hd_uint8 = 1920*1080*sizeof(std::uint8_t);
    managed_shared_memory segment(create_only, "VectorSharedMemory", size_full_hd_uint8);

    //Initialize shared memory STL-compatible allocator
    const SharedMemoryAllocator alloc_inst (segment.get_segment_manager());

    //Construct a vector named "ImageVector" in shared memory with argument alloc_inst
    ImageVector *myvector = segment.construct<ImageVector>("ImageVector")(alloc_inst);

    try
    {
        //Set size
        mutex_shared_memory.truncate(sizeof(ProcessSynchronizationQueue));

        //Map the whole shared memory in this process
        mapped_region region(mutex_shared_memory, read_write);

        //Get the address of the mapped region
        void *region_address = region.get_address();

        //Construct the shared structure in memory
        ProcessSynchronizationQueue *mutex_data = new (region_address) ProcessSynchronizationQueue;

        bool keep_looping = true;
        do
        {
            scoped_lock<interprocess_mutex> lock(mutex_data->mutex_);
            if(mutex_data->message_available_)
            {
                mutex_data->condition_full_.wait(lock);
            }


            delta_pose = msr::airlib::Pose(msr::airlib::Vector3r(-0.01f, 0.0f, 0.01f), msr::airlib::Quaternionr(1.0f, 0.0f, 0.0f, 0.0f));
            new_pose = new_pose - delta_pose;

            client->SetPositionAndOrientation(new_pose);

            std::cout << "Collided? Answer: " << client->HasCollided() << std::endl;

            std::vector<std::pair<StereoImageType,msr::airlib::VehicleCameraBase::ImageResponse>> received_image_response_vector = client->RequestStereoImagesAndDepthImage();

            mutex_data->number_remaining_sub_messages_ = static_cast<std::uint8_t>(received_image_response_vector.size());
                        
            while(mutex_data->number_remaining_sub_messages_ > 1)
            {
                const unsigned current_image_index = mutex_data->number_remaining_sub_messages_ - 1;
                if(mutex_data->message_available_)
                {
                    std::cout << "Waiting" << std::endl;
                    mutex_data->condition_full_.wait(lock);
                }
                
                myvector->clear();
                
                for(int i = 0; i < received_image_response_vector.at(current_image_index).second.image_data_uint8.size(); ++i)
                {
                    myvector->push_back(received_image_response_vector.at(current_image_index).second.image_data_uint8.at(i));
                }
                
                //Fill additional information on image - see https://answers.ros.org/question/195979/creating-sensor_msgsimage-from-scratch/
                mutex_data->message_image_information_.header_stamp_sec_ = 0;
                mutex_data->message_image_information_.header_stamp_nsec_ = 0;
                mutex_data->message_image_information_.header_seq_ = 0;
                mutex_data->message_image_information_.header_frame_id_ = "";
                mutex_data->message_image_information_.image_height_ = received_image_response_vector.at(current_image_index).second.height;
                mutex_data->message_image_information_.image_width_ = received_image_response_vector.at(current_image_index).second.width;
                mutex_data->message_image_information_.image_encoding_ = "rgba8";
                mutex_data->message_image_information_.image_is_bigendian_ = false;
                mutex_data->message_image_information_.type_ = StereoImageType::Unknown;
                mutex_data->message_image_information_.image_step_ = 4 * received_image_response_vector.at(current_image_index).second.height;
                
                //Notify to the other process that there is a message
                mutex_data->condition_empty_.notify_one();
                
                std::cout << "mutex_data->number_remaining_sub_messages_ = " << unsigned(mutex_data->number_remaining_sub_messages_) << std::endl;
                std::cout << "myvector->size() = " << myvector->size() << std::endl;
                std::cout << "myvector->at(0) = " << unsigned(myvector->at(0)) << std::endl;

                //Mark message buffer as full
                mutex_data->message_available_ = true;
            }
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
        }
        while(keep_looping);
    }
    catch(interprocess_exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }

return 0;
}