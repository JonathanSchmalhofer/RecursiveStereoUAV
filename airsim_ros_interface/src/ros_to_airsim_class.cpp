///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RosToAirSim Implementation
///

#include "ros_to_airsim_class.h"

namespace ros_to_airsim
{

RosToAirSimClass::RosToAirSimClass(std::string const& addr)
    : zmq_context_(1)
    , zmq_subscriber_(zmq_context_, ZMQ_SUB)
{
    zmq_subscriber_.setsockopt(ZMQ_IDENTITY, "AirSimToRosSubscriber", 5);
    zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq_subscriber_.setsockopt(ZMQ_RCVTIMEO, 5000);
    zmq_subscriber_.connect(addr);
    
    // Header
    image_header_seq_           = 0;
    image_header_stamp_sec_     = 0;
    image_header_stamp_nsec_    = 0;
    image_header_frame_id_      = "";
    // Image
    image_height_               = 0;
    image_width_                = 0;
    image_encoding_             = "";
    image_is_bigendian_         = 0;
    image_step_                 = 0;
    image_data_                 = NULL;
    image_data_size_            = 0;
    image_type_                 = 0;
}
    
RosToAirSimClass::~RosToAirSimClass()
{
    free(image_data_);
}

int8_t RosToAirSimClass::ReceivedMessage()
{
    zmq::message_t zmq_received_message;
    zmq_subscriber_.recv(&zmq_received_message);
    
    int8_t return_value = 0;

    /*
    flatbuffers::FlatBufferBuilder fbb;
    airsim_to_ros::ImageBuilder builder(fbb);

    // Verify the message contents are trustworthy.
    flatbuffers::Verifier verifier((const unsigned char *)zmq_received_message.data(),
                                                          zmq_received_message.size());
    int8_t return_value = 0;


    if (zmq_received_message.size() > 0)
    {
      // Make sure the message has valid data.
      if (airsim_to_ros::VerifyImageBuffer(verifier))
      {
          auto flatbuffer_image_rcvd = airsim_to_ros::GetImage(zmq_received_message.data());

          // Header
          image_header_seq_           = flatbuffer_image_rcvd->header()->seq();
          image_header_stamp_sec_     = flatbuffer_image_rcvd->header()->stamp()->sec();
          image_header_stamp_nsec_    = flatbuffer_image_rcvd->header()->stamp()->nsec();
          image_header_frame_id_      = flatbuffer_image_rcvd->header()->frame_id()->c_str();
          // Image
          image_height_               = flatbuffer_image_rcvd->height();
          image_width_                = flatbuffer_image_rcvd->width();
          image_encoding_             = flatbuffer_image_rcvd->encoding()->c_str();
          image_is_bigendian_         = flatbuffer_image_rcvd->is_bigendian();
          image_step_                 = flatbuffer_image_rcvd->step();
          image_data_                 = (std::uint8_t*)realloc(image_data_, zmq_received_message.size());
          image_data_size_            = image_step_ * image_height_;
          memcpy(image_data_, flatbuffer_image_rcvd->data(), image_data_size_);
          image_type_                 = flatbuffer_image_rcvd->type();
          return_value                = 1;
      }
      else
      {
          return_value                = -1;
      }
    }*/

    return return_value;
}
    
uint32_t RosToAirSimClass::GetImageHeaderSeq()
{
    return image_header_seq_;
}
    
uint32_t RosToAirSimClass::GetImageHeaderStampSec()
{
    return image_header_stamp_sec_;
}

uint32_t RosToAirSimClass::GetImageHeaderStampNsec()
{
    return image_header_stamp_nsec_;
}

std::string RosToAirSimClass::GetImageHeaderFrameid()
{
    return image_header_frame_id_;
}

uint32_t RosToAirSimClass::GetImageHeight()
{
    return image_height_;
}

uint32_t RosToAirSimClass::GetImageWidth()
{
    return image_width_;
}

std::string RosToAirSimClass::GetImageEncoding()
{
    return image_encoding_;
}

uint8_t RosToAirSimClass::GetImageIsBigendian()
{
    return image_is_bigendian_;
}

uint32_t RosToAirSimClass::GetImageStep()
{
    return image_step_;
}

std::uint8_t* RosToAirSimClass::GetImageData()
{
    return image_data_;
}
    
size_t RosToAirSimClass::GetImageDataSize()
{
    return image_data_size_;
}
    
int8_t RosToAirSimClass::GetImageType()
{
    return image_type_;
}
}  // namespace js_airsim_to_ros_library

