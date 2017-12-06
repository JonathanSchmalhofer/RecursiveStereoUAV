///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Trajectory3DPoint to AirSim Implementation
///

#include "js_trajectory_controller_node/trajectory_point_to_airsim_class.h"

namespace js_trajectory_controller_node
{

Trajectory3DPointToAirSimClass::Trajectory3DPointToAirSimClass(std::string const& addr)
    : zmq_context_(1)
    , zmq_publisher_(zmq_context_, ZMQ_PUB)
{
    zmq_publisher_.bind(addr);
    
    // Trajectory3DPointStamped
    trajectory3dpoint_timestamp_sec_        = 0;
    trajectory3dpoint_timestamp_nsec_       = 0;
    trajectory3dpoint_pose_position_x_      = 0;
    trajectory3dpoint_pose_position_y_      = 0;
    trajectory3dpoint_pose_position_z_      = 0;
    trajectory3dpoint_pose_orientation_x_   = 0;
    trajectory3dpoint_pose_orientation_y_   = 0;
    trajectory3dpoint_pose_orientation_z_   = 0;
    trajectory3dpoint_pose_orientation_w_   = 0;
        
    flatbuffers::FlatBufferBuilder fbb;
    
    /*
    // Image.header.stamp
    airsim_to_ros::time message_time(ros_image.header.stamp.sec,ros_image.header.stamp.nsec);
    // Image.header
    auto header = airsim_to_ros::CreateHeader(
        fbb, 
        ros_image.header.seq,
        &message_time,
        fbb.CreateString(ros_image.header.frame_id));
    fbb.Finish(header);
    // Image
    std::vector<std::uint8_t> image_data(3 * 640 * 480);
    auto image = airsim_to_ros::CreateImage(
        fbb, 
        0, 
        header, 
        ros_image.height,
        ros_image.width,
        fbb.CreateString(ros_image.encoding),
        ros_image.is_bigendian,
        ros_image.step,
        fbb.CreateVector<std::uint8_t>(ros_image.data));
    fbb.Finish(image);

    ROS_INFO("   Sending Image");
    */
    int buffersize = fbb.GetSize();
    zmq::message_t image_msg(buffersize);
    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffersize);
    zmq_publisher_.send(image_msg);
    //ROS_INFO("   Image Sent (%d Bytes)", buffersize);
}
    
Trajectory3DPointToAirSimClass::~Trajectory3DPointToAirSimClass()
{
}
}  // namespace js_airsim_to_ros_library

