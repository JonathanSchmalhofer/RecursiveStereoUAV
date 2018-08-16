///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief zmq publisher and subscriber reading stereo images from filesystem (offline)
///
#include "airsim_offline_image_provider.hpp"
#include "ros_to_airsim_class.h"
#include "InputParser.hpp"
#include "kitti_utils.hpp"

bool endless_looping                    = false;
bool debug_mode_activated               = false;
bool camera_calibration_mode_activated  = false;
bool no_zeromq_forwarding               = false;
std::string address                     = "tcp://127.0.0.1:6677";
std::string left_folder                 = "";
std::string right_folder                = "";
std::string oxts_folder                 = "";
kitti_utils::KittiUtils utils;

double pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw;

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
    if (input_parser.cmdOptionExists("-o"))
    {
        oxts_folder = input_parser.getCmdOption("-o");
    }
    if (input_parser.cmdOptionExists("--oxts-folder"))
    {
        oxts_folder = input_parser.getCmdOption("--oxts-folder");
    }
    
    std::vector<std::string> left_file_list  = kitti_utils::GetFileList(left_folder,  ".png");
    std::vector<std::string> right_file_list = kitti_utils::GetFileList(right_folder, ".png");
       
    //  We send updates via this socket
    zmq::context_t context(1);
    
    // Publisher - Images
    zmq::socket_t publish_socket_image = zmq::socket_t(context, ZMQ_PUB);
    publish_socket_image.bind("tcp://*:5676");
    // Publisher - Pose
    zmq::socket_t publish_socket_pose = zmq::socket_t(context, ZMQ_PUB);
    publish_socket_pose.bind("tcp://*:5677");
    
    // Subscriber
    if (debug_mode_activated)
    {
        std::cout << "Subscribing to " << address << std::endl;
    }
    ros_to_airsim::RosToAirSimClass ros_to_airsim(address);
    
    std::vector<long>                       timestamps;
    std::vector<std::string>                filelist_oxts;
    double                                  scale;
    boost::numeric::ublas::matrix<double>   Tr_0(4, 4);
    boost::numeric::ublas::matrix<double>   Tr_0_inv(4, 4);
    utils.LoadOxtsLitePaths(oxts_folder, timestamps, filelist_oxts);
    scale    = utils.GetScale(filelist_oxts);
    Tr_0     = utils.GetTr0(filelist_oxts, scale);
    bool inversion_succeeded = InvertMatrix(Tr_0, Tr_0_inv);
    assert(inversion_succeeded==true);
	if (debug_mode_activated)
	{
		std::cout << "   scale   = " << scale << std::endl;
		std::cout << "    Tr_0   = " << Tr_0 << std::endl;
		std::cout << "inv(Tr_0)  = " << Tr_0_inv << std::endl;
	}
    
    bool keep_looping           = true;
    std::uint32_t send_counter  = 0;
    std::uint32_t image_counter = 0;
    
    assert(left_file_list.size() == right_file_list.size());
    assert(left_file_list.size() == filelist_oxts.size());
    
    
    while(keep_looping)
    {
        // copy image into flatbuffers
        flatbuffers::FlatBufferBuilder fbb_left;
        flatbuffers::FlatBufferBuilder fbb_right;
        flatbuffers::FlatBufferBuilder fbb_pose;
        
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
        
        // PoseMessage.header
        auto header_pose = airsim_to_ros::CreateHeader(
            fbb_pose, 
            send_counter,               // header_seq_
            &message_time,
            fbb_pose.CreateString("")   // header_frame_id_
            );
        fbb_pose.Finish(header_pose);
        
        unsigned width_left, width_right, height_left, height_right;
        std::vector<unsigned char> image_png_left, image_png_right;
        int buffersize_left, buffersize_right, buffersize_pose;
        
        // Get image for frame = idx
        decodeOneStep(left_file_list.at(image_counter),  width_left,  height_left,  image_png_left);
        decodeOneStep(right_file_list.at(image_counter), width_right, height_right, image_png_right);
        // Get pose for frame = idx
        kitti_utils::oxts_data datapoint = utils.GetOxtsData(filelist_oxts, timestamps, image_counter);
        boost::numeric::ublas::matrix<double> pose = utils.ConvertOxtsToPose(datapoint, scale, Tr_0_inv);
        if (debug_mode_activated)
        {
            std::cout << "   pose(idx=" << image_counter << ") = " << pose << std::endl;
        }
        utils.GetEulerAngles(pose, pose_roll, pose_pitch, pose_yaw);
        utils.GetPosition(   pose, pose_x,    pose_y,     pose_z);
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
        
        // PoseMessage.position
        airsim_to_ros::Point pose_position(
            pose_x, // x
            pose_y, // y
            pose_z  // z
        );
        
        // PoseMessage.orientation
        airsim_to_ros::Orientation pose_orientation(
            pose_roll,  // roll
            pose_pitch, // pitch
            pose_yaw    // yaw
        );
        
        // Pose
        auto pose_message = airsim_to_ros::CreatePoseMessage(
            fbb_pose,
            header_pose,
            &pose_position,     // position
            &pose_orientation); // orientation
        fbb_pose.Finish(pose_message);
        
        // Copy flatbuffers into zmq message and send it
        buffersize_left = fbb_left.GetSize();
        zmq::message_t image_msg_left(buffersize_left);
        memcpy((void *)image_msg_left.data(), fbb_left.GetBufferPointer(), buffersize_left);
        
        buffersize_right = fbb_right.GetSize();
        zmq::message_t image_msg_right(buffersize_right);
        memcpy((void *)image_msg_right.data(), fbb_right.GetBufferPointer(), buffersize_right);
        
        buffersize_pose = fbb_pose.GetSize();
        zmq::message_t pose_msg(buffersize_pose);
        memcpy((void *)pose_msg.data(), fbb_pose.GetBufferPointer(), buffersize_pose);
        
        // Send left and right
        publish_socket_pose.send(pose_msg);
        publish_socket_image.send(image_msg_left);
        publish_socket_image.send(image_msg_right);
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
            std::cout << " " << std::endl;
            std::cout << "P O S E" << std::endl;
            std::cout << "Sent flatbuffer message via zmq: " << buffersize_pose << std::endl;
            std::cout << "    x: "     << pose_x     << std::endl;
            std::cout << "    y: "     << pose_y     << std::endl;
            std::cout << "    z: "     << pose_z     << std::endl;
            std::cout << "    roll: "  << pose_roll  << std::endl;
            std::cout << "    pitch: " << pose_pitch << std::endl;
            std::cout << "    yaw: "   << pose_yaw   << std::endl;
            std::cout << " " << std::endl;
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