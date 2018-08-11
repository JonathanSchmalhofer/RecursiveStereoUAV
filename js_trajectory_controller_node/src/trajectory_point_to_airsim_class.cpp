///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
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
}

void Trajectory3DPointToAirSimClass::SendTrajectory3DPoint()
{
  flatbuffers::FlatBufferBuilder fbb;

  // Trajectory3DPointStamped.timestamp
  ros_to_airsim::time trajectory_timestamp(trajectory3dpoint_timestamp_sec_,trajectory3dpoint_timestamp_nsec_);

  // Trajectory3DPointStamped.pose
  ros_to_airsim::Point trajectory_position(trajectory3dpoint_pose_position_x_, trajectory3dpoint_pose_position_y_, trajectory3dpoint_pose_position_z_);
  ros_to_airsim::Quaternion trajectory_orientation(trajectory3dpoint_pose_orientation_x_, trajectory3dpoint_pose_orientation_y_, trajectory3dpoint_pose_orientation_z_, trajectory3dpoint_pose_orientation_w_);
  ros_to_airsim::Pose trajectory_pose(trajectory_position, trajectory_orientation);

  // Trajectory3DPointStamped
  auto trajectory3dpoint = ros_to_airsim::CreateTrajectory3DPointStamped(
      fbb,
        &trajectory_timestamp,
        &trajectory_pose
      );
  fbb.Finish(trajectory3dpoint);

  int buffersize = fbb.GetSize();
  zmq::message_t trajectory3dpoint_msg(buffersize);
  memcpy((void *)trajectory3dpoint_msg.data(), fbb.GetBufferPointer(), buffersize);
  zmq_publisher_.send(trajectory3dpoint_msg);
}
    
Trajectory3DPointToAirSimClass::~Trajectory3DPointToAirSimClass()
{
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointTimeStampSec(std::uint32_t sec)
{
    trajectory3dpoint_timestamp_sec_ = sec;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointTimeStampNsec(std::uint32_t nsec)
{
    trajectory3dpoint_timestamp_nsec_ = nsec;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPosePositionX(double x)
{
    trajectory3dpoint_pose_position_x_ = x;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPosePositionY(double y)
{
    trajectory3dpoint_pose_position_y_ = y;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPosePositionZ(double z)
{
    trajectory3dpoint_pose_position_z_ = z;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPoseOrientationX(double x)
{
    trajectory3dpoint_pose_orientation_x_ = x;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPoseOrientationY(double y)
{
    trajectory3dpoint_pose_orientation_y_ = y;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPoseOrientationZ(double z)
{
    trajectory3dpoint_pose_orientation_z_ = z;
}

void Trajectory3DPointToAirSimClass::SetTrajectory3DPointPoseOrientationW(double w)
{
    trajectory3dpoint_pose_orientation_w_ = w;
}
}  // namespace js_airsim_to_ros_library

