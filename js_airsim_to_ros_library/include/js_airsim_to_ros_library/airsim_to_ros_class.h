///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief Class to receive Data from AirSim and make available in ROS as ROS-Message
///
#ifndef JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
#define JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_

#include <cstdint>
#include <zeromq_cpp/zmq.hpp>
//#include <js_messages/Image_generated.h>
//#include <js_messages/PoseMessageRoot_generated.h>
#include <js_messages/StereoImagePose_generated.h>

namespace js_airsim_to_ros_library
{

class AirSimToRosClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    AirSimToRosClass(std::string const& addr);
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~AirSimToRosClass();
	
    /// @brief Connect to addr - useful to "reconnect" if connection was lost or could not be established on creation of object
    void Connect(std::string const& addr);
    
    // @brief Returns true if a new, full message was received via ZeroMq, false otherwise
    int8_t ReceivedMessage();
    
    // @brief Returns the attribute StereoImagePose.Header.seq
    uint32_t GetHeaderSeq();
    
    // @brief Returns the attribute StereoImagePose.stamp.sec
    uint32_t GetHeaderStampSec();
    
    // @brief Returns the attribute StereoImagePose.stamp.nsec
    uint32_t GetHeaderStampNsec();
    
    // @brief Returns the attribute StereoImagePose.frame_id
    std::string GetHeaderFrameid();
    
    // @brief Returns the attribute StereoImagePose.leftheight
    uint32_t GetLeftHeight();
    
    // @brief Returns the attribute StereoImagePose.leftwidth
    uint32_t GetLeftWidth();
    
    // @brief Returns the attribute StereoImagePose.leftencoding
    std::string GetLeftEncoding();
    
    // @brief Returns the attribute StereoImagePose.leftis_bigendian
    uint8_t GetLeftIsBigendian();
    
    // @brief Returns the attribute StereoImagePose.leftstep
    uint32_t GetLeftStep();
    
    // @brief Returns pointer to the attribute StereoImagePose.leftdata
    std::uint8_t* GetLeftData();
    
    // @brief Returns the size of the left image data.
    size_t GetLeftDataSize();
    
    // @brief Returns the attribute StereoImagePose.right.height
    uint32_t GetRightHeight();
    
    // @brief Returns the attribute StereoImagePose.right.width
    uint32_t GetRightWidth();
    
    // @brief Returns the attribute StereoImagePose.right.encoding
    std::string GetRightEncoding();
    
    // @brief Returns the attribute StereoImagePose.right.is_bigendian
    uint8_t GetRightIsBigendian();
    
    // @brief Returns the attribute StereoImagePose.right.step
    uint32_t GetRightStep();
    
    // @brief Returns pointer to the attribute StereoImagePose.right.data
    std::uint8_t* GetRightData();
    
    // @brief Returns the size of the StereoImagePose.right data.
    size_t GetRightDataSize();
    
    // @brief Returns the attribute PoseMessage.position.x
    double GetPosePositionX();
    
    // @brief Returns the attribute PoseMessage.position.y
    double GetPosePositionY();
    
    // @brief Returns the attribute PoseMessage.position.z
    double GetPosePositionZ();
    
    // @brief Returns the attribute PoseMessage.orientation.roll
    double GetPoseOrientationRoll();
    
    // @brief Returns the attribute PoseMessage.orientation.pitch
    double GetPoseOrientationPitch();
    
    // @brief Returns the attribute PoseMessage.orientation.yaw
    double GetPoseOrientationYaw();

private:    
    /// @brief A ZeroMq context object encapsulating functionality dealing with the initialisation and termination.
    zmq::context_t zmq_context_;
    
    /// @brief A ZeroMq socket for subscribing to incoming messages.
    zmq::socket_t zmq_subscriber_;

    /// @brief Holds the attribute StereoImagePose.Header.seq of the last received message.
    uint32_t header_seq_;

    /// @brief Holds the attribute StereoImagePose.Header.stamp.sec of the last received message.
    uint32_t header_stamp_sec_;

    /// @brief Holds the attribute StereoImagePose.Header.stamp.sec of the last received message.
    uint32_t header_stamp_nsec_;

    /// @brief Holds the attribute StereoImagePose.Header.frame_id of the last received message.
    std::string header_frame_id_;

    /// @brief Holds the attribute StereoImagePose.left.height of the last received message.
    uint32_t left_height_;

    /// @brief Holds the attribute StereoImagePose.left.width of the last received message.
    uint32_t left_width_;

    /// @brief Holds the attribute StereoImagePose.left.encoding of the last received message.
    std::string left_encoding_;

    /// @brief Holds the attribute StereoImagePose.left.is_bigendian of the last received message.
    uint8_t left_is_bigendian_;

    /// @brief Holds the attribute StereoImagePose.left.step of the last received message.
    uint32_t left_step_;
    
    /// @brief Holds the attribute StereoImagePose.left.data of the last received message.
    std::uint8_t* left_data_;
    
    /// @brief Holds the size of the StereoImagePose.left data (which is step * height).
    size_t left_data_size_;

    /// @brief Holds the attribute StereoImagePose.right.height of the last received message.
    uint32_t right_height_;

    /// @brief Holds the attribute StereoImagePose.right.width of the last received message.
    uint32_t right_width_;

    /// @brief Holds the attribute StereoImagePose.right.encoding of the last received message.
    std::string right_encoding_;

    /// @brief Holds the attribute StereoImagePose.right.is_bigendian of the last received message.
    uint8_t right_is_bigendian_;

    /// @brief Holds the attribute StereoImagePose.right.step of the last received message.
    uint32_t right_step_;
    
    /// @brief Holds the attribute StereoImagePose.right.data of the last received message.
    std::uint8_t* right_data_;
    
    /// @brief Holds the size of the StereoImagePose.right data (which is step * height).
    size_t right_data_size_;

    /// @brief Holds the attribute PoseMessage.position.x of the last received message.
    double pose_position_x_;

    /// @brief Holds the attribute PoseMessage.position.y of the last received message.
    double pose_position_y_;

    /// @brief Holds the attribute PoseMessage.position.z of the last received message.
    double pose_position_z_;

    /// @brief Holds the attribute PoseMessage.orientation.roll of the last received message.
    double pose_orientation_roll_;

    /// @brief Holds the attribute PoseMessage.orientation.pitch of the last received message.
    double pose_orientation_pitch_;

    /// @brief Holds the attribute PoseMessage.orientation.yaw of the last received message.
    double pose_orientation_yaw_;
};
}  // namespace js_airsim_to_ros_library

#endif  // JS_AIRSIM_TO_ROS_LIBRARY_AIRSIM_TO_ROS_CLASS_H_
