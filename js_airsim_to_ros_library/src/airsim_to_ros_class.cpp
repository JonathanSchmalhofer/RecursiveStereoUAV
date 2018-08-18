///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief AirSimToRos Implementation
///

#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

namespace js_airsim_to_ros_library
{

AirSimToRosClass::AirSimToRosClass(std::string const& addr)
    : zmq_context_(1)
    , zmq_subscriber_(zmq_context_, ZMQ_SUB)
{
    zmq_subscriber_.setsockopt(ZMQ_IDENTITY, "AirSimToRosSubscriber", 5);
    zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq_subscriber_.setsockopt(ZMQ_RCVTIMEO, 5000);
    zmq_subscriber_.connect(addr);
    
    // Header
    header_seq_                 = 0;
    header_stamp_sec_           = 0;
    header_stamp_nsec_          = 0;
    header_frame_id_            = "";
    // Left
    left_height_                = 0;
    left_width_                 = 0;
    left_encoding_              = "";
    left_is_bigendian_          = 0;
    left_step_                  = 0;
    left_data_                  = NULL;
    left_data_size_             = 0;
    // Right
    right_height_               = 0;
    right_width_                = 0;
    right_encoding_             = "";
    right_is_bigendian_         = 0;
    right_step_                 = 0;
    right_data_                 = NULL;
    right_data_size_            = 0;
    
    // Pose.position
    pose_position_x_            = 0;
    pose_position_y_            = 0;
    pose_position_z_            = 0;
    // Pose.orientation
    pose_orientation_roll_      = 0;
    pose_orientation_pitch_     = 0;
    pose_orientation_yaw_       = 0;
}

void AirSimToRosClass::Connect(std::string const& addr)
{
	zmq_subscriber_.connect(addr);
}
    
AirSimToRosClass::~AirSimToRosClass()
{
    free(left_data_);
    free(right_data_);
}

int8_t AirSimToRosClass::ReceivedMessage()
{
    zmq::message_t zmq_received_message;
    zmq_subscriber_.recv(&zmq_received_message);

    flatbuffers::FlatBufferBuilder fbb;
    airsim_to_ros::StereoImagePoseBuilder builder(fbb);

    // Verify the message contents are trustworthy.
    flatbuffers::Verifier verifier((const unsigned char *)zmq_received_message.data(),
                                                          zmq_received_message.size());
    int8_t return_value = 0;


    if (zmq_received_message.size() > 0)
    {
      // Make sure the message has valid data.
      if (airsim_to_ros::VerifyStereoImagePoseBuffer(verifier))
      {
          auto flatbuffer_rcvd = airsim_to_ros::GetStereoImagePose(zmq_received_message.data());

          // Header
          header_seq_                 = flatbuffer_rcvd->header()->seq();
          header_stamp_sec_           = flatbuffer_rcvd->header()->stamp()->sec();
          header_stamp_nsec_          = flatbuffer_rcvd->header()->stamp()->nsec();
          header_frame_id_            = flatbuffer_rcvd->header()->frame_id()->c_str();
          // Left
          left_height_               = flatbuffer_rcvd->left()->height();
          left_width_                = flatbuffer_rcvd->left()->width();
          left_encoding_             = flatbuffer_rcvd->left()->encoding()->c_str();
          left_is_bigendian_         = flatbuffer_rcvd->left()->is_bigendian();
          left_step_                 = flatbuffer_rcvd->left()->step();
          left_data_                 = (std::uint8_t*)realloc(left_data_, left_step_ * left_height_);//zmq_received_message.size());
          left_data_size_            = left_step_ * left_height_;
          memcpy(left_data_, flatbuffer_rcvd->left()->data(), left_data_size_);
          // Right
          right_height_               = flatbuffer_rcvd->right()->height();
          right_width_                = flatbuffer_rcvd->right()->width();
          right_encoding_             = flatbuffer_rcvd->right()->encoding()->c_str();
          right_is_bigendian_         = flatbuffer_rcvd->right()->is_bigendian();
          right_step_                 = flatbuffer_rcvd->right()->step();
          right_data_                 = (std::uint8_t*)realloc(right_data_, right_step_ * right_height_);//zmq_received_message.size());
          right_data_size_            = right_step_ * right_height_;
          memcpy(right_data_, flatbuffer_rcvd->right()->data(), right_data_size_);
          
          // position
          pose_position_x_            = flatbuffer_rcvd->pose()->position()->x();
          pose_position_y_            = flatbuffer_rcvd->pose()->position()->y();
          pose_position_z_            = flatbuffer_rcvd->pose()->position()->z();
          // orientation
          pose_orientation_roll_      = flatbuffer_rcvd->pose()->orientation()->roll();
          pose_orientation_pitch_     = flatbuffer_rcvd->pose()->orientation()->pitch();
          pose_orientation_yaw_       = flatbuffer_rcvd->pose()->orientation()->yaw();
          return_value                = 1;
      }
      else
      {
          return_value                = -1;
      }
    }

    return return_value;
}
    
uint32_t AirSimToRosClass::GetHeaderSeq()
{
    return header_seq_;
}
    
uint32_t AirSimToRosClass::GetHeaderStampSec()
{
    return header_stamp_sec_;
}

uint32_t AirSimToRosClass::GetHeaderStampNsec()
{
    return header_stamp_nsec_;
}

std::string AirSimToRosClass::GetHeaderFrameid()
{
    return header_frame_id_;
}

uint32_t AirSimToRosClass::GetLeftHeight()
{
    return left_height_;
}

uint32_t AirSimToRosClass::GetLeftWidth()
{
    return left_width_;
}

std::string AirSimToRosClass::GetLeftEncoding()
{
    return left_encoding_;
}

uint8_t AirSimToRosClass::GetLeftIsBigendian()
{
    return left_is_bigendian_;
}

uint32_t AirSimToRosClass::GetLeftStep()
{
    return left_step_;
}

std::uint8_t* AirSimToRosClass::GetLeftData()
{
    return left_data_;
}
    
size_t AirSimToRosClass::GetLeftDataSize()
{
    return left_data_size_;
}

uint32_t AirSimToRosClass::GetRightHeight()
{
    return right_height_;
}

uint32_t AirSimToRosClass::GetRightWidth()
{
    return right_width_;
}

std::string AirSimToRosClass::GetRightEncoding()
{
    return right_encoding_;
}

uint8_t AirSimToRosClass::GetRightIsBigendian()
{
    return right_is_bigendian_;
}

uint32_t AirSimToRosClass::GetRightStep()
{
    return right_step_;
}

std::uint8_t* AirSimToRosClass::GetRightData()
{
    return right_data_;
}
    
size_t AirSimToRosClass::GetRightDataSize()
{
    return right_data_size_;
}

double AirSimToRosClass::GetPosePositionX()
{
    return pose_position_x_;
}

double AirSimToRosClass::GetPosePositionY()
{
    return pose_position_y_;
}

double AirSimToRosClass::GetPosePositionZ()
{
    return pose_position_z_;
}

double AirSimToRosClass::GetPoseOrientationRoll()
{
    return pose_orientation_roll_;
}

double AirSimToRosClass::GetPoseOrientationPitch()
{
    return pose_orientation_pitch_;
}

double AirSimToRosClass::GetPoseOrientationYaw()
{
    return pose_orientation_yaw_;
}
}  // namespace js_airsim_to_ros_library

