#include "StereoImageRpcClient.hpp"
#include "airsim_rpc_client.hpp"
#include "InputParser.hpp"

bool debug_mode_activated = false;


int main(int argc, const char *argv[])
{
    InputParser input_parser(argc, argv);

    if (input_parser.cmdOptionExists("-d") || input_parser.cmdOptionExists("--debug"))
    {
        debug_mode_activated = true;
    }

    StereoImageRpcClient *client = new StereoImageRpcClient();    
    client->Connect();
    msr::airlib::Pose initial_pose = client->GetPositionAndOrientation();

    msr::airlib::Pose delta_pose, new_pose = initial_pose;

    //Create a shared memory object.
    shared_memory_object mutex_shared_memory(create_only, "MutexSharedMemory", read_write);

    //Create a new segment with given name and size
    managed_shared_memory segment(create_only, "VectorSharedMemory", size_full_hd_uint8);

    //Initialize shared memory STL-compatible allocator
    const SharedMemoryAllocator alloc_inst (segment.get_segment_manager());

    //Construct a vector named "ImageVector" in shared memory with argument alloc_inst
    ImageVector *image_vector = segment.construct<ImageVector>("ImageVector")(alloc_inst);

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

            // TODO: set new pose as it was reported back from trajectory planner
            delta_pose = msr::airlib::Pose(msr::airlib::Vector3r(-0.01f, 0.0f, 0.01f), msr::airlib::Quaternionr(1.0f, 0.0f, 0.0f, 0.0f));
            new_pose = new_pose - delta_pose;
            client->SetPositionAndOrientation(new_pose);

            // get process queue of stereo images from client
            auto received_image_response_vector = client->RequestStereoImagesAndDepthImage();
            mutex_data->number_remaining_sub_messages_ = static_cast<std::uint8_t>(received_image_response_vector.size());
            
            // Loop remaining messages in process queue
            while(mutex_data->number_remaining_sub_messages_ > 1)
            {
                if(mutex_data->message_available_)
                {
                    if (debug_mode_activated)
                    {
                        std::cout << "Waiting" << std::endl;
                    }
                    mutex_data->condition_full_.wait(lock);
                }
                const unsigned current_image_index = (mutex_data->number_remaining_sub_messages_ - 1);
                
                // extract image data
                image_vector->clear();                
                for(int i = 0; i < received_image_response_vector.at(current_image_index).second.image_data_uint8.size(); ++i)
                {
                    image_vector->push_back(received_image_response_vector.at(current_image_index).second.image_data_uint8.at(i));
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
                mutex_data->message_image_information_.type_ = received_image_response_vector.at(current_image_index).first;
                mutex_data->message_image_information_.image_step_ = 4 * received_image_response_vector.at(current_image_index).second.height;
                
                //Notify to the other process that there is a message
                mutex_data->condition_empty_.notify_one();
                if (debug_mode_activated)
                {
                    std::cout << "mutex_data->message_image_information_.type_ = " << signed(mutex_data->message_image_information_.type_) << std::endl;
                    std::cout << "mutex_data->number_remaining_sub_messages_ = " << unsigned(mutex_data->number_remaining_sub_messages_) << std::endl;
                    std::cout << "image_vector->size() = " << image_vector->size() << std::endl;
                    std::cout << "image_vector->at(0) = " << unsigned(image_vector->at(0)) << std::endl;
                }

                //Mark message buffer as full
                mutex_data->message_available_ = true;
            }
            if (debug_mode_activated)
            {
                std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
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