///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Class to receive Data from ROS and make them available in AirSim
///
#ifndef ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_
#define ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_

#include <cstdint>
#include <zeromq_cpp/zmq.hpp>
//#include "Image_generated.h"

namespace ros_to_airsim
{

class RosToAirSimClass
{
public:
    /// @brief Initializes a AirSimToRos class instance.
    RosToAirSimClass(std::string const& addr);
    
    /// @brief Default Destructor to free dynamically allocated memory of received messages.
    ~RosToAirSimClass();
    
    // @brief Returns true if a new, full message was received via ZeroMq, false otherwise
    int8_t ReceivedMessage();
    
    // @brief Returns the attribute Image.Header.seq
    uint32_t GetImageHeaderSeq();
    
    // @brief Returns the attribute Image.stamp.sec
    uint32_t GetImageHeaderStampSec();
    
    // @brief Returns the attribute Image.stamp.nsec
    uint32_t GetImageHeaderStampNsec();
    
    // @brief Returns the attribute Image.frame_id
    std::string GetImageHeaderFrameid();
    
    // @brief Returns the attribute Image.height
    uint32_t GetImageHeight();
    
    // @brief Returns the attribute Image.width
    uint32_t GetImageWidth();
    
    // @brief Returns the attribute Image.encoding
    std::string GetImageEncoding();
    
    // @brief Returns the attribute Image.is_bigendian
    uint8_t GetImageIsBigendian();
    
    // @brief Returns the attribute Image.step
    uint32_t GetImageStep();
    
    // @brief Returns pointer to the attribute Image.data
    std::uint8_t* GetImageData();
    
    // @brief Returns the size of the image data.
    size_t GetImageDataSize();
    
    // @brief Returns attribute Image.type
    int8_t GetImageType();

private:    
    /// @brief A ZeroMq context object encapsulating functionality dealing with the initialisation and termination.
    zmq::context_t zmq_context_;
    
    /// @brief A ZeroMq socket for subscribing to incoming messages.
    zmq::socket_t zmq_subscriber_;

    /// @brief Holds the attribute Image.Header.seq of the last received message.
    uint32_t image_header_seq_;

    /// @brief Holds the attribute Image.Header.stamp.sec of the last received message.
    uint32_t image_header_stamp_sec_;

    /// @brief Holds the attribute Image.Header.stamp.sec of the last received message.
    uint32_t image_header_stamp_nsec_;

    /// @brief Holds the attribute Image.Header.frame_id of the last received message.
    std::string image_header_frame_id_;

    /// @brief Holds the attribute Image.height of the last received message.
    uint32_t image_height_;

    /// @brief Holds the attribute Image.width of the last received message.
    uint32_t image_width_;

    /// @brief Holds the attribute Image.encoding of the last received message.
    std::string image_encoding_;

    /// @brief Holds the attribute Image.is_bigendian of the last received message.
    uint8_t image_is_bigendian_;

    /// @brief Holds the attribute Image.step of the last received message.
    uint32_t image_step_;
    
    /// @brief Holds the attribute Image.data of the last received message.
    std::uint8_t* image_data_;
    
    /// @brief Holds the size of the image data (which is step * height).
    size_t image_data_size_;    

    /// @brief Holds the attribute Image.type of the last received message.
    int8_t image_type_;
};
}  // namespace ros_to_airsim

#endif  // ROS_TO_AIRSIM_ROS_TO_AIRSIM_CLASS_H_
