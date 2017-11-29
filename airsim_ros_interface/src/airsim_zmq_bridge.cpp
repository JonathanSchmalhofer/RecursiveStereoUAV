#include "StereoImageRpcClient.hpp"
#include "airsim_zmq_bridge.hpp"
#include "InputParser.hpp"

bool debug_mode_activated = false;

int main(int argc, const char *argv[])
{
    InputParser input_parser(argc, argv);

    if (input_parser.cmdOptionExists("-d") || input_parser.cmdOptionExists("--debug"))
    {
        debug_mode_activated = true;
    }
    
    //Create a shared memory object.
    shared_memory_object mutex_shared_memory(open_only, "MutexSharedMemory", read_write);

    //  We send updates via this socket
    zmq::context_t context(1);
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
                ImageVector *image_vector = segment.find<ImageVector>("ImageVector").first;
                
                std::vector<msr::airlib::VehicleCameraBase::ImageResponse> received_image_response_vector;
                
                while(mutex_data->number_remaining_sub_messages_ > 0)
                {
                    if(!mutex_data->message_available_)
                    {
                        if (debug_mode_activated)
                        {
                            std::cout << "Waiting" << std::endl;
                        }
                        mutex_data->condition_empty_.wait(lock);
                    }
                    
                    if (debug_mode_activated)
                    {
                        std::cout << "mutex_data->number_remaining_sub_messages_ = " << unsigned(mutex_data->number_remaining_sub_messages_) << std::endl;
                    }
                    
                    msr::airlib::VehicleCameraBase::ImageResponse sub_message_image;
                    for(int i = 0; i < image_vector->size(); ++i)
                    {
                        sub_message_image.image_data_uint8.push_back(image_vector->at(i));
                    }
                    received_image_response_vector.push_back(sub_message_image);
                    if (debug_mode_activated)
                    {
                        std::cout << "image_vector->size() = " << image_vector->size() << std::endl;
                        std::cout << "image_vector->at(0) = " << unsigned(image_vector->at(0)) << std::endl;
                    }
                    
                    // copy image into flatbuffers
                    flatbuffers::FlatBufferBuilder fbb;
                    
                    // Image.header.stamp
                    airsim_to_ros::time message_time(
                        mutex_data->message_image_information_.header_stamp_sec_,
                        mutex_data->message_image_information_.header_stamp_nsec_);
                    
                    // Image.header
                    auto header = airsim_to_ros::CreateHeader(
                        fbb, 
                        mutex_data->message_image_information_.header_seq_,
                        &message_time,
                        fbb.CreateString(mutex_data->message_image_information_.header_frame_id_));
                    fbb.Finish(header);
                    
                    // Image
                    auto image = airsim_to_ros::CreateImage(
                        fbb,
                        static_cast<std::int8_t>(mutex_data->message_image_information_.type_),
                        header,
                        mutex_data->message_image_information_.image_height_,
                        mutex_data->message_image_information_.image_width_,
                        fbb.CreateString(mutex_data->message_image_information_.image_encoding_),
                        mutex_data->message_image_information_.image_is_bigendian_,
                        mutex_data->message_image_information_.image_step_,
                        fbb.CreateVector<std::uint8_t>(sub_message_image.image_data_uint8));
                    fbb.Finish(image);
                    
                    // Copy flatbuffers into zmq message and send it
                    int buffersize = fbb.GetSize();
                    zmq::message_t image_msg(buffersize);
                    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffersize);
                    publishSocket.send(image_msg);
                    
                    if (debug_mode_activated)
                    {
                        std::cout << "Sent flatbuffer message via zmq: " << buffersize << std::endl;
                        std::cout << "    buffersize: " << buffersize << std::endl;
                        std::cout << "    type: " << unsigned(mutex_data->message_image_information_.type_) << std::endl;
                        std::cout << "    height: " << unsigned(mutex_data->message_image_information_.image_height_) << std::endl;
                        std::cout << "    width: " << unsigned(mutex_data->message_image_information_.image_width_) << std::endl;
                        std::cout << "    step: " << unsigned(mutex_data->message_image_information_.image_step_) << std::endl;
                        std::cout << "    encoding: " << mutex_data->message_image_information_.image_encoding_ << std::endl;
                    }
                    
                    // Decrease remaining number in queue and notify the other process that the buffer has been emptied
                    mutex_data->number_remaining_sub_messages_--;
                    mutex_data->message_available_ = false;
                    mutex_data->condition_full_.notify_one();
                }
                if (debug_mode_activated)
                {
                    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                }
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