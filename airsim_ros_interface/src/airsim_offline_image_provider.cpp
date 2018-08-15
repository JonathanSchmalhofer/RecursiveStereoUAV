///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief zmq publisher and subscriber reading stereo images from filesystem (offline)
///
#include "airsim_offline_image_provider.hpp"
#include "ros_to_airsim_class.h"
#include "InputParser.hpp"

bool endless_looping                    = false;
bool debug_mode_activated               = false;
bool camera_calibration_mode_activated  = false;
bool no_zeromq_forwarding               = false;
std::string address                     = "tcp://127.0.0.1:6677";
std::string left_folder                 = "";
std::string right_folder                = "";

// Source for GetFileList(): https://gist.github.com/vivithemage/9517678
std::vector<std::string> GetFileList(const std::string& path)
{
    std::vector<std::string> file_list;
    if (!path.empty())
    {
        boost::filesystem::path apk_path(path);
        boost::filesystem::recursive_directory_iterator end;

        for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const boost::filesystem::path cp = (*i);
            if (".png" == boost::filesystem::extension(cp.string()))
            {
                file_list.push_back(cp.string());
            }
        }
    }
    std::sort(file_list.begin(), file_list.end());
    return file_list;
}

// Copied from https://raw.githubusercontent.com/lvandeve/lodepng/master/examples/example_decode.cpp
void decodeOneStep(const std::string filename, unsigned &width, unsigned &height, std::vector<unsigned char> &image)
{
  //decode
  unsigned error = lodepng::decode(image, width, height, filename.c_str());

  //if there's an error, display it
  if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;

  //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
}

int main(int argc, const char *argv[])
{
    InputParser input_parser(argc, argv);
    
    if (input_parser.cmdOptionExists("-d") || input_parser.cmdOptionExists("--debug"))
    {
        debug_mode_activated = true;
    }
    if (input_parser.cmdOptionExists("-e") || input_parser.cmdOptionExists("--endless"))
    {
        endless_looping = true;
    }
    if (input_parser.cmdOptionExists("-a"))
    {
        address = input_parser.getCmdOption("-a");
    }
    if (input_parser.cmdOptionExists("--address"))
    {
        address = input_parser.getCmdOption("--address");
    }
    if (input_parser.cmdOptionExists("-cc") || input_parser.cmdOptionExists("--camera-calibration"))
    {
        std::cout << "Camera calibration mode activated\n";
        camera_calibration_mode_activated = true;
    }
    if (input_parser.cmdOptionExists("-l"))
    {
        left_folder = input_parser.getCmdOption("-l");
        std::cout << "PNG files (left) will be imported from:" << std::endl;
        std::cout << "    " << left_folder << std::endl;
    }
    if (input_parser.cmdOptionExists("--left-folder"))
    {
        left_folder = input_parser.getCmdOption("--left-folder");
        std::cout << "PNG files (left) will be imported from:" << std::endl;
        std::cout << "    " << left_folder << std::endl;
    }
    if (input_parser.cmdOptionExists("-r"))
    {
        right_folder = input_parser.getCmdOption("-r");
        std::cout << "PNG files (right) will be imported from:" << std::endl;
        std::cout << "    " << right_folder << std::endl;
    }
    if (input_parser.cmdOptionExists("--right-folder"))
    {
        right_folder = input_parser.getCmdOption("--right-folder");
        std::cout << "PNG files (right) will be imported from:" << std::endl;
        std::cout << "    " << right_folder << std::endl;
    }
    
    std::vector<std::string> left_file_list  = GetFileList(left_folder);
    std::vector<std::string> right_file_list = GetFileList(right_folder);
       
    //  We send updates via this socket
    zmq::context_t context(1);
    
    // Publisher
    zmq::socket_t publish_socket = zmq::socket_t(context, ZMQ_PUB);
    publish_socket.bind("tcp://*:5676");
    
    // Subscriber
    if (debug_mode_activated)
    {
        std::cout << "Subscribing to " << address << std::endl;
    }
    ros_to_airsim::RosToAirSimClass ros_to_airsim(address);
    
    bool keep_looping           = true;
    std::uint32_t send_counter  = 0;
    std::uint32_t image_counter = 0;
    
    assert(left_file_list.size() == right_file_list.size());
    
    while(keep_looping)
    {
        // copy image into flatbuffers
        flatbuffers::FlatBufferBuilder fbb_left;
        flatbuffers::FlatBufferBuilder fbb_right;
        
        // Image.header.stamp
        airsim_to_ros::time message_time(
            0, // header_stamp_sec_
            0  // header_stamp_nsec_
        );
        
        // Image.header left
        auto header_left = airsim_to_ros::CreateHeader(
            fbb_left, 
            send_counter,               // header_seq_
            &message_time,
            fbb_left.CreateString("")   // header_frame_id_
            );
        fbb_left.Finish(header_left);
        
        // Image.header right
        auto header_right = airsim_to_ros::CreateHeader(
            fbb_right, 
            send_counter,               // header_seq_
            &message_time,
            fbb_right.CreateString("")  // header_frame_id_
            );
        fbb_right.Finish(header_right);
        
        unsigned width_left, width_right, height_left, height_right;
        std::vector<unsigned char> image_png_left, image_png_right;
        int buffersize_left, buffersize_right;
        
        decodeOneStep(left_file_list.at(image_counter),  width_left,  height_left,  image_png_left);
        decodeOneStep(right_file_list.at(image_counter), width_right, height_right, image_png_right);
        image_counter++;
        
        if(image_counter >= left_file_list.size())
        {
            if(endless_looping)
            {
                image_counter = 0;
            }
            else
            {
                keep_looping = false;
            }
        }
        
        // Image Left
        auto image_left = airsim_to_ros::CreateImage(
            fbb_left,
            static_cast<std::int8_t>(StereoImageType::LeftStereoImage), // type_
            header_left,
            height_left,                        // height_
            width_left,                         // width_
            fbb_left.CreateString("rgba8"),     // encoding_
            false,                              // is_bigendian_
            4 * width_left,                     // step_ (4 * width)
            fbb_left.CreateVector<std::uint8_t>(image_png_left)); // For reinterpret_cast<> see: https://stackoverflow.com/questions/16260033/reinterpret-cast-between-char-and-stduint8-t-safe
        fbb_left.Finish(image_left);
       
        // Image Left
        auto image_right = airsim_to_ros::CreateImage(
            fbb_right,
            static_cast<std::int8_t>(StereoImageType::RightStereoImage), // type_
            header_right,
            height_right,                       // height_
            width_right,                        // width_
            fbb_right.CreateString("rgba8"),    // encoding_
            false,                              // is_bigendian_
            4 * width_right,                    // step_ (4 * width)
            fbb_right.CreateVector<std::uint8_t>(image_png_right)); // For reinterpret_cast<> see: https://stackoverflow.com/questions/16260033/reinterpret-cast-between-char-and-stduint8-t-safe
        fbb_right.Finish(image_right);
        
        // Copy flatbuffers into zmq message and send it
        buffersize_left = fbb_left.GetSize();
        zmq::message_t image_msg_left(buffersize_left);
        memcpy((void *)image_msg_left.data(), fbb_left.GetBufferPointer(), buffersize_left);
        
        buffersize_right = fbb_right.GetSize();
        zmq::message_t image_msg_right(buffersize_right);
        memcpy((void *)image_msg_right.data(), fbb_right.GetBufferPointer(), buffersize_right);
        
        // Send left and right
        publish_socket.send(image_msg_left);
        publish_socket.send(image_msg_right);
        send_counter++;
        
        if (debug_mode_activated)
        {
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
            std::cout << "L E F T" << std::endl;
            std::cout << "Sent flatbuffer message via zmq: " << buffersize_left << std::endl;
            std::cout << "    buffersize: " << buffersize_left << std::endl;
            std::cout << "    type: "       << unsigned(StereoImageType::LeftStereoImage) << std::endl;
            std::cout << "    height: "     << unsigned(height_left) << std::endl;
            std::cout << "    width: "      << unsigned(width_left) << std::endl;
            std::cout << "    step: "       << unsigned(4 * width_left) << std::endl;
            std::cout << "    encoding: "   << "rgba8" << std::endl;
            std::cout << " " << std::endl;
            std::cout << "R I G H T" << std::endl;
            std::cout << "Sent flatbuffer message via zmq: " << buffersize_right << std::endl;
            std::cout << "    buffersize: " << buffersize_right << std::endl;
            std::cout << "    type: "       << unsigned(StereoImageType::RightStereoImage) << std::endl;
            std::cout << "    height: "     << unsigned(height_right) << std::endl;
            std::cout << "    width: "      << unsigned(width_right) << std::endl;
            std::cout << "    step: "       << unsigned(4 * width_right) << std::endl;
            std::cout << "    encoding: "   << "rgba8" << std::endl;
            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
        }
        
        uint8_t wait_loop_counter = 0;
        int8_t  received_message  = 0;
        do
        {
            received_message = ros_to_airsim.ReceivedMessage();
            if (debug_mode_activated)
            {
                std::cout << "Waiting..." << std::endl;
            }
            if (wait_loop_counter > 10)
            {
                if (debug_mode_activated)
                {
                    std::cout << "!!!!! Timeout while waiting for new trajectory !!!!!" << std::endl;
                    std::cout << "Trying to reconnect to " << address << " ..." << std::endl;
                }
                ros_to_airsim.Connect(address);
                break;
            }
            if (-1 == received_message)
            {
                if (debug_mode_activated)
                {
                    std::cout << "!!!!! Received Message of wrong size or format !!!!!" << std::endl;
                }
                break;
            }
            
            wait_loop_counter++;
        }
        while(0 == received_message);
        
        /*
        // Write last received data into shared memory
        mutex_data->trajectory_pose_.position_.x_ = ros_to_airsim.GetTrajectory3DPointPosePositionX();
        mutex_data->trajectory_pose_.position_.y_ = ros_to_airsim.GetTrajectory3DPointPosePositionY();
        mutex_data->trajectory_pose_.position_.z_ = ros_to_airsim.GetTrajectory3DPointPosePositionZ();
        mutex_data->trajectory_pose_.orientation_.x_ = ros_to_airsim.GetTrajectory3DPointPoseOrientationX();
        mutex_data->trajectory_pose_.orientation_.y_ = ros_to_airsim.GetTrajectory3DPointPoseOrientationY();
        mutex_data->trajectory_pose_.orientation_.z_ = ros_to_airsim.GetTrajectory3DPointPoseOrientationZ();
        mutex_data->trajectory_pose_.orientation_.w_ = ros_to_airsim.GetTrajectory3DPointPoseOrientationW();
        */
    }
    
    std::cout << "Finished" << std::endl;

    return 0;
}