///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
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
    
    // Trajectory3DPoint
    trajectory3dpoint_time_sec_             = 0;
    trajectory3dpoint_time_nsec_            = 0;
    trajectory3dpoint_pose_position_x_      = 0;
    trajectory3dpoint_pose_position_y_      = 0;
    trajectory3dpoint_pose_position_z_      = 0;
    trajectory3dpoint_pose_orientation_x_   = 0;
    trajectory3dpoint_pose_orientation_y_   = 0;
    trajectory3dpoint_pose_orientation_z_   = 0;
    trajectory3dpoint_pose_orientation_w_   = 0;
}
    
RosToAirSimClass::~RosToAirSimClass()
{
}

int8_t RosToAirSimClass::ReceivedMessage()
{
    zmq::message_t zmq_received_message;
    zmq_subscriber_.recv(&zmq_received_message);
    
    
    flatbuffers::FlatBufferBuilder fbb;
    ros_to_airsim::Trajectory3DPointStampedBuilder builder(fbb);

    // Verify the message contents are trustworthy.
    flatbuffers::Verifier verifier((const unsigned char *)zmq_received_message.data(),
                                                          zmq_received_message.size());
    int8_t return_value = 0;

    if (zmq_received_message.size() > 0)
    {
      // Make sure the message has valid data.
      if (ros_to_airsim::VerifyTrajectory3DPointStampedBuffer(verifier))
      {
          auto flatbuffer_trajectory3d_rcvd = ros_to_airsim::GetTrajectory3DPointStamped(zmq_received_message.data());

          // Trajectory3DPoint
          trajectory3dpoint_time_sec_           = flatbuffer_trajectory3d_rcvd->timestamp()->sec();
          trajectory3dpoint_time_nsec_          = flatbuffer_trajectory3d_rcvd->timestamp()->nsec();
          trajectory3dpoint_pose_position_x_    = flatbuffer_trajectory3d_rcvd->pose()->position().x();
          trajectory3dpoint_pose_position_y_    = flatbuffer_trajectory3d_rcvd->pose()->position().y();
          trajectory3dpoint_pose_position_z_    = flatbuffer_trajectory3d_rcvd->pose()->position().z();
          trajectory3dpoint_pose_orientation_x_ = flatbuffer_trajectory3d_rcvd->pose()->orientation().x();
          trajectory3dpoint_pose_orientation_y_ = flatbuffer_trajectory3d_rcvd->pose()->orientation().y();
          trajectory3dpoint_pose_orientation_z_ = flatbuffer_trajectory3d_rcvd->pose()->orientation().z();
          trajectory3dpoint_pose_orientation_w_ = flatbuffer_trajectory3d_rcvd->pose()->orientation().w();
          
          return_value                = 1;
      }
      else
      {
          return_value                = -1;
      }
    }

    return return_value;
}
    
uint32_t RosToAirSimClass::GetTrajectory3DPointTimeSec()
{
    return trajectory3dpoint_time_sec_;
}

uint32_t RosToAirSimClass::GetTrajectory3DPointTimeNsec()
{
    return trajectory3dpoint_time_nsec_;
}

double RosToAirSimClass::GetTrajectory3DPointPosePositionX()
{
    return trajectory3dpoint_pose_position_x_;
}

double RosToAirSimClass::GetTrajectory3DPointPosePositionY()
{
    return trajectory3dpoint_pose_position_y_;
}

double RosToAirSimClass::GetTrajectory3DPointPosePositionZ()
{
    return trajectory3dpoint_pose_position_z_;
}

double RosToAirSimClass::GetTrajectory3DPointPoseOrientationX()
{
    return trajectory3dpoint_pose_orientation_x_;
}

double RosToAirSimClass::GetTrajectory3DPointPoseOrientationY()
{
    return trajectory3dpoint_pose_orientation_y_;
}

double RosToAirSimClass::GetTrajectory3DPointPoseOrientationZ()
{
    return trajectory3dpoint_pose_orientation_z_;
}

double RosToAirSimClass::GetTrajectory3DPointPoseOrientationW()
{
    return trajectory3dpoint_pose_orientation_w_;
}
}  // namespace js_airsim_to_ros_library

