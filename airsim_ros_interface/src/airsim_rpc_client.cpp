///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief rpc client receiving stereo images from AirSim and writing them in shared memory queue
///
#include "StereoImageRpcClient.hpp"
#include "airsim_rpc_client.hpp"
#include "InputParser.hpp"

bool debug_mode_activated = false;
bool camera_calibration_mode_activated = false;
bool camera_calibration_automatic_recording = false;
bool initial_pose_set = false;
msr::airlib::Pose initial_pose, new_pose, last_recorded_pose;

bool significantDifferenceInPose(msr::airlib::Pose one, msr::airlib::Pose two)
{
    bool significant_change = false;
    double threshold = 2.0f;
    double difference = std::sqrt(std::pow(one.position.x()-two.position.x(), 2) + std::pow(one.position.y()-two.position.y(), 2) + std::pow(one.position.z()-two.position.z(), 2));
    if (difference>threshold)
    {
        std::cout << "Significant change of " << difference << "\n";
        significant_change = true;
    }
    return significant_change;
}

void encodeOneStep(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
{
  //Encode the image
  unsigned error = lodepng::encode(filename, image, width, height);
  

  //if there's an error, display it
  if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}

int main(int argc, const char *argv[])
{
    InputParser input_parser(argc, argv);

    if (input_parser.cmdOptionExists("-d") || input_parser.cmdOptionExists("--debug"))
    {
        debug_mode_activated = true;
    }
    if (input_parser.cmdOptionExists("-cc") || input_parser.cmdOptionExists("--camera-calibration"))
    {
        std::cout << "Camera calibration mode activated\n";
        camera_calibration_mode_activated = true;
    }
    if (input_parser.cmdOptionExists("-cca") || input_parser.cmdOptionExists("--camera-calibration-automatic"))
    {
        std::cout << "Camera calibration mode with automatic recording activated\n";
        camera_calibration_mode_activated = true;
        camera_calibration_automatic_recording = true;
    }

    StereoImageRpcClient *client = new StereoImageRpcClient();    
    client->Connect();
    if (false == initial_pose_set)
    {
        initial_pose = client->GetPositionAndOrientation();
        new_pose = initial_pose;
        last_recorded_pose = initial_pose;
        initial_pose_set = true;
    }

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
        mutex_shared_memory.truncate(sizeof(airsim_to_ros::ProcessSynchronizationQueue));

        //Map the whole shared memory in this process
        mapped_region region(mutex_shared_memory, read_write);

        //Get the address of the mapped region
        void *region_address = region.get_address();

        //Construct the shared structure in memory
        airsim_to_ros::ProcessSynchronizationQueue *mutex_data = new (region_address) airsim_to_ros::ProcessSynchronizationQueue;
        
        //Set initial pose in queue
        mutex_data->initial_pose_.position_.x_ = initial_pose.position.x();
        mutex_data->initial_pose_.position_.y_ = initial_pose.position.y();
        mutex_data->initial_pose_.position_.z_ = initial_pose.position.z();
        mutex_data->initial_pose_.orientation_.x_ = initial_pose.orientation.x();
        mutex_data->initial_pose_.orientation_.y_ = initial_pose.orientation.y();
        mutex_data->initial_pose_.orientation_.z_ = initial_pose.orientation.z();
        mutex_data->initial_pose_.orientation_.w_ = initial_pose.orientation.w();

        std::uint32_t loop_counter = 0;
        bool keep_looping = true;
        do
        {
            scoped_lock<interprocess_mutex> lock(mutex_data->mutex_);

            // After last loop, wait until queue has been emptied
            if(mutex_data->message_available_)
            {
                if (debug_mode_activated)
                {
                    std::cout << "Waiting for queue to be emptied and trajectory point to be set" << std::endl;
                }    
                // this condition will only be triggered once a new trajectory is available
                mutex_data->condition_trajectory_.wait(lock);
                if (debug_mode_activated)
                {
                    std::cout << "New trajectory point received" << std::endl;
                }
                if (camera_calibration_mode_activated)
                {
                    if (true == camera_calibration_automatic_recording)
                    {
                        std::cout << "\n" << "Waiting for significant change in pose" << std::endl;
                        // wait for position change
                        msr::airlib::Pose current_pose;
                        do
                        {
                            current_pose = client->GetPositionAndOrientation();
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
                        while(false==significantDifferenceInPose(current_pose, last_recorded_pose));
                        last_recorded_pose = current_pose;
                    }
                    else
                    {
                        WaitForAnyKeyPress();
                    }
                }
                loop_counter++;
            }
            
            // if the trajectory point has been updated in the queue, get it
            new_pose = msr::airlib::Pose(
                msr::airlib::Vector3r(
                    mutex_data->trajectory_pose_.position_.x_,
                    mutex_data->trajectory_pose_.position_.y_,
                    mutex_data->trajectory_pose_.position_.z_),
                msr::airlib::Quaternionr(
                    mutex_data->trajectory_pose_.orientation_.w_,
                    mutex_data->trajectory_pose_.orientation_.x_,
                    mutex_data->trajectory_pose_.orientation_.y_,
                    mutex_data->trajectory_pose_.orientation_.z_));
            if (false == camera_calibration_mode_activated)
            {
                client->SetPositionAndOrientation(new_pose);
            }

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
                        std::cout << "  Waiting for queue to be emptied for next image" << std::endl;
                    }
                    mutex_data->condition_full_.wait(lock);
                }
                const unsigned current_image_index = (mutex_data->number_remaining_sub_messages_ - 1);
                
                // extract image data
                image_vector->clear();
                // Only copy if image needs to be forwarded
                if (false == camera_calibration_automatic_recording)
                {
                    for(int i = 0; i < received_image_response_vector.at(current_image_index).second.image_data_uint8.size(); ++i)
                    {
                        image_vector->push_back(received_image_response_vector.at(current_image_index).second.image_data_uint8.at(i));
                    }
                }
                
                
                if (camera_calibration_mode_activated)
                {
                    std::ostringstream filename_stringstream;                
                    switch(received_image_response_vector.at(current_image_index).first)
                    {
                        case StereoImageType::LeftStereoImage:
                                filename_stringstream << "left_";
                                break;
                        case StereoImageType::RightStereoImage:
                                filename_stringstream << "right_";
                                break;
                        default:
                                filename_stringstream << "unknown_";
                                break;
                    }
                    filename_stringstream << std::setfill('0') << std::setw(5) << loop_counter << ".png";
                    
                    std::string filename_image = filename_stringstream.str();
                    
                    if (    StereoImageType::LeftStereoImage == received_image_response_vector.at(current_image_index).first
                        ||  StereoImageType::RightStereoImage == received_image_response_vector.at(current_image_index).first
                        )
                    {
                        EncodeAndSaveImageAsPng(
                            filename_image.c_str(),
                            received_image_response_vector.at(current_image_index).second.image_data_uint8,
                            received_image_response_vector.at(current_image_index).second.width,
                            received_image_response_vector.at(current_image_index).second.height
                        );
                    }
                    
                    //Fill additional information on image - see https://answers.ros.org/question/195979/creating-sensor_msgsimage-from-scratch/
                    mutex_data->message_image_information_.header_stamp_sec_ = 0;
                    mutex_data->message_image_information_.header_stamp_nsec_ = 0;
                    mutex_data->message_image_information_.header_seq_ = loop_counter;
                    mutex_data->message_image_information_.header_frame_id_ = "";
                    mutex_data->message_image_information_.image_height_ = received_image_response_vector.at(current_image_index).second.height;
                    mutex_data->message_image_information_.image_width_ = received_image_response_vector.at(current_image_index).second.width;
                    mutex_data->message_image_information_.image_encoding_ = "rgba8";
                    mutex_data->message_image_information_.image_is_bigendian_ = false;
                    mutex_data->message_image_information_.type_ = received_image_response_vector.at(current_image_index).first;
                    mutex_data->message_image_information_.image_step_ = 4 * received_image_response_vector.at(current_image_index).second.width;
                }
                
                //Notify to the other process that there is a message
                mutex_data->condition_empty_.notify_one();
                if (debug_mode_activated)
                {
                    std::cout << "mutex_data->message_image_information_.type_ = " << signed(mutex_data->message_image_information_.type_) << std::endl;
                    std::cout << "mutex_data->number_remaining_sub_messages_ = " << unsigned(mutex_data->number_remaining_sub_messages_) << std::endl;
                    std::cout << "image_vector->size() = " << image_vector->size() << std::endl;
                    if (image_vector->size() > 0)
                    {
                        std::cout << "image_vector->at(0) = " << unsigned(image_vector->at(0)) << std::endl;
                    }
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

void WaitForAnyKeyPress()
{
    do 
    {
        std::cout << "\n" << "Press enter to take a camera calibration snapshot\n";
    } while (std::cin.get() != '\n');
}

void EncodeAndSaveImageAsPng(const char* filename, std::vector<std::uint8_t>& image, unsigned width, unsigned height)
{
  std::vector<std::uint8_t> png;
  lodepng::State state; //optionally customize this one

  unsigned error = lodepng::encode(png, image, width, height, state);
  if(!error) lodepng::save_file(png, filename);

  //if there's an error, display it
  if(error) std::cout << "png encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}