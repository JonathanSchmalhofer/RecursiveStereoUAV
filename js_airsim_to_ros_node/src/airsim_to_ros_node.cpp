///
/// @file
/// @copyright Copyright (C) 2017-2018, Jonathan Bryan Schmalhofer
///
/// @brief Node to wrap airsim_to_ros to receive Data from AirSim and make available in ROS as ROS-Message
///
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <js_common/common.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

#include <tf/tf.h>

sensor_msgs::Image airsim_image_left_msg, airsim_image_right_msg;
geometry_msgs::Pose airsim_pose_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting AirSim to ros node");
    
    // Get Parameters from ROS Parameter Server
    bool verbose;
    std::string ip_airsim;
    std::string port_airsim_to_ros;
    js_common::TryGetParameter("/recursivestereo/parameters/verbose",                  verbose,                  false);
    js_common::TryGetParameter("/recursivestereo/parameters/ip_airsim",                ip_airsim,                "127.0.0.1");
    js_common::TryGetParameter("/recursivestereo/parameters/port_airsim_to_ros",       port_airsim_to_ros,       "5676");
    
    // Subscribe to IP of Host running AirSim
    std::string connection_info = "tcp://" + ip_airsim + ":" + port_airsim_to_ros;
    ROS_INFO("Subscribing for Images to:    %s", connection_info.c_str());
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros(connection_info);
    
    ROS_INFO("Created airsim_to_ros");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher left_stereoimage_chatter  = image_transport.advertise("/airsim/left/image_raw", 1);
    image_transport::Publisher right_stereoimage_chatter = image_transport.advertise("/airsim/right/image_raw", 1);
    ros::Publisher publisher_heartbeat                   = node_handle.advertise<std_msgs::Bool>("/airsim/heartbeat", 1);
    ros::Publisher pose_chatter                          = node_handle.advertise<geometry_msgs::Pose>("/airsim/pose", 1);

    std::uint32_t last_timestamp_sent_ms   = 0;
    std::uint32_t waiting_counter          = 0;
    std::uint32_t waiting_threshold        = 10;
    while (ros::ok())
    {
        if ( verbose == true )
        {
            ROS_INFO("Waiting for data");
        }
        if ( waiting_counter >= waiting_threshold )
        {
            if ( verbose == true )
            {
                ROS_INFO("Waiting too long - trying to reconnect to   %s", connection_info.c_str());
            }
            // Signalize within ROS, that we are still alive
            std_msgs::Bool msg;
            msg.data = true;
            publisher_heartbeat.publish(msg);
            // try to reconnect
            airsim_to_ros.Connect(connection_info);
            // Reset counter
            waiting_counter = 0;
        }
        waiting_counter++;
        int8_t received_return_value  = airsim_to_ros.ReceivedMessage();
        if (1 == received_return_value)
        {
            waiting_counter_pose = 0;
            if ( verbose == true )
            {
                ROS_INFO("StereoImagePose received");
            }
            // Header for Left ROS Message
            airsim_image_left_msg.header.seq         = airsim_to_ros.GetHeaderSeq();
            airsim_image_left_msg.header.stamp.sec   = airsim_to_ros.GetHeaderStampSec();
            airsim_image_left_msg.header.stamp.nsec  = airsim_to_ros.GetHeaderStampNsec();
            airsim_image_left_msg.header.frame_id    = airsim_to_ros.GetHeaderFrameid();
            // Image Left
            airsim_image_left_msg.height             = airsim_to_ros.GetLeftHeight();
            airsim_image_left_msg.width              = airsim_to_ros.GetLeftWidth();
            airsim_image_left_msg.encoding           = airsim_to_ros.GetLeftEncoding();
            airsim_image_left_msg.is_bigendian       = airsim_to_ros.GetLeftIsBigendian();
            airsim_image_left_msg.step               = airsim_to_ros.GetLeftStep();
            airsim_image_left_msg.data.resize(airsim_to_ros.GetLeftDataSize());
            memcpy((char*)(&airsim_image_left_msg.data[0]), airsim_to_ros.GetLeftData(), airsim_to_ros.GetLeftDataSize());
            
            
            // Header for Left ROS Message
            airsim_image_right_msg.header.seq         = airsim_to_ros.GetHeaderSeq();
            airsim_image_right_msg.header.stamp.sec   = airsim_to_ros.GetHeaderStampSec();
            airsim_image_right_msg.header.stamp.nsec  = airsim_to_ros.GetHeaderStampNsec();
            airsim_image_right_msg.header.frame_id    = airsim_to_ros.GetHeaderFrameid();
            // Image Right
            airsim_image_right_msg.height             = airsim_to_ros.GetRightHeight();
            airsim_image_right_msg.width              = airsim_to_ros.GetRightWidth();
            airsim_image_right_msg.encoding           = airsim_to_ros.GetRightEncoding();
            airsim_image_right_msg.is_bigendian       = airsim_to_ros.GetRightIsBigendian();
            airsim_image_right_msg.step               = airsim_to_ros.GetRightStep();
            airsim_image_right_msg.data.resize(airsim_to_ros.GetRightDataSize());
            memcpy((char*)(&airsim_image_right_msg.data[0]), airsim_to_ros.GetRightData(), airsim_to_ros.GetRightDataSize());
            
            // Pose
            airsim_pose_msg.position.x          = airsim_to_ros.GetPosePositionX();
            airsim_pose_msg.position.y          = airsim_to_ros.GetPosePositionY();
            airsim_pose_msg.position.z          = airsim_to_ros.GetPosePositionZ();
            
            tf::Quaternion q = tf::createQuaternionFromRPY(airsim_to_ros.GetPoseOrientationRoll(),
                                                           airsim_to_ros.GetPoseOrientationPitch(),
                                                           airsim_to_ros.GetPoseOrientationYaw());  // Create this quaternion from roll/pitch/yaw (in radians)
            airsim_pose_msg.orientation.x       = q[0];
            airsim_pose_msg.orientation.y       = q[1];
            airsim_pose_msg.orientation.z       = q[2];
            airsim_pose_msg.orientation.w       = q[3];
            
            
            if ( verbose == true )
            {
                ROS_INFO("Header");
                ROS_INFO("  header.seq %d", airsim_to_ros.GetHeaderSeq());
                ROS_INFO("  header.stamp.sec %d", airsim_to_ros.GetHeaderStampSec());
                ROS_INFO("  header.stamp.nsec %d", airsim_to_ros.GetHeaderStampNsec());
                ROS_INFO("  header.frame_id %s", airsim_to_ros.GetHeaderFrameid().c_str());
                ROS_INFO("Left Image");
                ROS_INFO("  Left.height %d", airsim_to_ros.GetLeftHeight());
                ROS_INFO("  Left.width %d", airsim_to_ros.GetLeftWidth());
                ROS_INFO("  Left.encoding %s", airsim_to_ros.GetLeftEncoding().c_str());
                ROS_INFO("  Left.is_bigendian %d", airsim_to_ros.GetLeftIsBigendian());
                ROS_INFO("  Left.step %d", airsim_to_ros.GetLeftStep());
                ROS_INFO("  size(Left.data) %d", static_cast<int>(airsim_to_ros.GetLeftDataSize()));
                ROS_INFO("Right Image");
                ROS_INFO("  Right.height %d", airsim_to_ros.GetRightHeight());
                ROS_INFO("  Right.width %d", airsim_to_ros.GetRightWidth());
                ROS_INFO("  Right.encoding %s", airsim_to_ros.GetRightEncoding().c_str());
                ROS_INFO("  Right.is_bigendian %d", airsim_to_ros.GetRightIsBigendian());
                ROS_INFO("  Right.step %d", airsim_to_ros.GetRightStep());
                ROS_INFO("  size(Right.data) %d", static_cast<int>(airsim_to_ros.GetRightDataSize()));
                ROS_INFO(" ");
                ROS_INFO("Pose");
                ROS_INFO("  Pose.position.x             %f", airsim_to_ros.GetPosePositionX());
                ROS_INFO("  Pose.position.y             %f", airsim_to_ros.GetPosePositionY());
                ROS_INFO("  Pose.position.z             %f", airsim_to_ros.GetPosePositionZ());
                ROS_INFO("  Pose.orientation.roll       %f", airsim_to_ros.GetPoseOrientationRoll());
                ROS_INFO("  Pose.orientation.pitch      %f", airsim_to_ros.GetPoseOrientationPitch());
                ROS_INFO("  Pose.orientation.yaw        %f", airsim_to_ros.GetPoseOrientationYaw());
            }
            
            std::uint32_t current_timestamp_ms = static_cast<std::uint32_t>((airsim_to_ros.GetHeaderStampSec() * 1e9 + airsim_to_ros.GetHeaderStampNsec())/1e6);
            
            if (
                    airsim_image_left_msg.header.stamp.sec  == airsim_image_right_msg.header.stamp.sec
                &&  airsim_image_left_msg.header.stamp.nsec == airsim_image_right_msg.header.stamp.nsec
                &&  current_timestamp_ms > last_timestamp_sent_ms
            )
            {
                if ( verbose == true )
                {
                    ROS_INFO("StereoImagePose forwarded");
                }
                pose_chatter.publish(airsim_pose_msg);
                left_stereoimage_chatter.publish(airsim_image_left_msg);
                right_stereoimage_chatter.publish(airsim_image_right_msg);
                last_timestamp_sent_ms = current_timestamp_ms;
            }
        }
        else if (-1 == received_return_value)
        {
            ROS_INFO("Invalid StereoImagePose received - did not forward");
        }
    }

    return 0;
}