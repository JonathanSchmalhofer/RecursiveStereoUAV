#include "StereoImageRpcClient.hpp"
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



int main(int argc, const char *argv[])
{
    //Create a shared memory object.
    shared_memory_object mutex_shared_memory(open_only, "MutexSharedMemory", read_write);
    
    zmq::context_t context(1);

    //  We send updates via this socket
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB);
    publishSocket.bind("tcp://*:5676");
    
    try
    {
        //Map the whole shared memory in this process
        mapped_region region(mutex_shared_memory, read_write);

        //Get the address of the mapped region
        void *region_address = region.get_address();

        //Obtain a pointer to the shared structure
        ProcessSynchronizationQueue *mutex_data = static_cast<ProcessSynchronizationQueue*>(region_address);
        
        bool keep_looping = true;
        do
        {
            scoped_lock<interprocess_mutex> lock(mutex_data->mutex_);
            if(!mutex_data->message_available_)
            {
                mutex_data->condition_empty_.wait(lock);
            }
            else
            {
                //Open the managed segment
                managed_shared_memory segment(open_only, "VectorSharedMemory");

                //Find the vector using the c-string name
                ImageVector *myvector = segment.find<ImageVector>("ImageVector").first;

                //std::cout << "myvector->size() = " << myvector->size() << std::endl;
                //std::cout << "myvector->at(0) = " << unsigned(myvector->at(0)) << std::endl;
                
                std::vector<msr::airlib::VehicleCameraBase::ImageResponse> received_image_response_vector;
                
                while(mutex_data->number_sub_messages_ > 0)
                {
                    if(!mutex_data->message_available_)
                    {
                        std::cout << "Waiting" << std::endl;
                        mutex_data->condition_empty_.wait(lock);
                    }
                    
                    std::cout << "mutex_data->number_sub_messages_ = " << unsigned(mutex_data->number_sub_messages_) << std::endl;
                    
                    msr::airlib::VehicleCameraBase::ImageResponse sub_message_image;
                    for(int i = 0; i < myvector->size(); ++i)
                    {
                        sub_message_image.image_data_uint8.push_back(myvector->at(i));
                    }
                    received_image_response_vector.push_back(sub_message_image);
                    std::cout << "myvector->size() = " << myvector->size() << std::endl;
                    std::cout << "myvector->at(0) = " << unsigned(myvector->at(0)) << std::endl;
                    
                    /////////////////////////////////////////////////
                    const std::uint8_t header_stamp_sec = 0;
                    const std::uint8_t header_stamp_nsec = 0;
                    const std::uint8_t header_seq = 0;
                    const std::string  header_frame_id = "";
                    const std::uint8_t  image_height = 0;
                    const std::uint8_t  image_width = 0;
                    const std::string  image_encoding = "";
                    const bool  image_is_bigendian = false;
                    const std::uint8_t  image_step = 0;
                    
                    flatbuffers::FlatBufferBuilder fbb;
                    
                    // Image.header.stamp
                    airsim_to_ros::time message_time(mutex_data->message_image_information_.header_stamp_sec_,mutex_data->message_image_information_.header_stamp_nsec_);
                    
                    // Image.header
                    auto header = airsim_to_ros::CreateHeader(
                        fbb, 
                        mutex_data->message_image_information_.header_seq_,
                        &message_time,
                        fbb.CreateString(mutex_data->message_image_information_.header_frame_id_));
                    fbb.Finish(header);
                    
                    // Image
                    //std::vector<std::uint8_t> image_data(3 * 640 * 480);
                    auto image = airsim_to_ros::CreateImage(
                        fbb, 
                        header, 
                        mutex_data->message_image_information_.image_height_,
                        mutex_data->message_image_information_.image_width_,
                        fbb.CreateString(mutex_data->message_image_information_.image_encoding_),
                        mutex_data->message_image_information_.image_is_bigendian_,
                        mutex_data->message_image_information_.image_step_,
                        fbb.CreateVector<std::uint8_t>(sub_message_image.image_data_uint8));
                    fbb.Finish(image);
                    
                    int buffersize = fbb.GetSize();
                    zmq::message_t image_msg(buffersize);
                    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffersize);
                    publishSocket.send(image_msg);
                    std::cout << "Sent flatbuffer message via zmq: " << buffersize << std::endl;
                    std::cout << "    buffersize: " << buffersize << std::endl;
                    std::cout << "    height: " << unsigned(mutex_data->message_image_information_.image_height_) << std::endl;
                    std::cout << "    width: " << unsigned(mutex_data->message_image_information_.image_width_) << std::endl;
                    std::cout << "    step: " << unsigned(mutex_data->message_image_information_.image_step_) << std::endl;
                    std::cout << "    encoding: " << mutex_data->message_image_information_.image_encoding_ << std::endl;
                    /////////////////////////////////////////////////
                    
                    //Notify the other process that the buffer is empty
                    mutex_data->number_sub_messages_--;
                    mutex_data->message_available_ = false;
                    mutex_data->condition_full_.notify_one();
                }
                std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                mutex_data->message_available_ = false;
                mutex_data->condition_full_.notify_one();
            }
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