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
    std::string port_airsim_to_ros_image;
    std::string port_airsim_to_ros_pose;
    js_common::TryGetParameter("/recursivestereo/parameters/verbose",                  verbose,                  false);
    js_common::TryGetParameter("/recursivestereo/parameters/ip_airsim",                ip_airsim,                "127.0.0.1");
    js_common::TryGetParameter("/recursivestereo/parameters/port_airsim_to_ros_image", port_airsim_to_ros_image, "5676");
    js_common::TryGetParameter("/recursivestereo/parameters/port_airsim_to_ros_pose",  port_airsim_to_ros_pose,  "5677");
    
    // Subscribe to IP of Host running AirSim
    std::string connection_info_image = "tcp://" + ip_airsim + ":" + port_airsim_to_ros_image;
    std::string connection_info_pose  = "tcp://" + ip_airsim + ":" + port_airsim_to_ros_pose;
    ROS_INFO("Subscribing for Images to:    %s", connection_info_image.c_str());
    ROS_INFO("Subscribing for Pose to:      %s", connection_info_pose.c_str());
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros_image(connection_info_image, js_airsim_to_ros_library::AirSimToRosType::ImageReceiver);
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros_pose(connection_info_pose,   js_airsim_to_ros_library::AirSimToRosType::PoseReceiver);
    
    ROS_INFO("Created airsim_to_ros_image");
    ROS_INFO("Created airsim_to_ros_pose");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher left_stereoimage_chatter  = image_transport.advertise("/airsim/left/image_raw", 1);
    image_transport::Publisher right_stereoimage_chatter = image_transport.advertise("/airsim/right/image_raw", 1);
    ros::Publisher publisher_heartbeat                   = node_handle.advertise<std_msgs::Bool>("/airsim/heartbeat", 1);
    ros::Publisher pose_chatter                          = node_handle.advertise<geometry_msgs::Pose>("/airsim/pose", 1);

    std::uint32_t last_sequence_sent       = 0;
    std::uint32_t waiting_counter_image    = 0;
    std::uint32_t waiting_counter_pose     = 0;
    std::uint32_t waiting_threshold        = 10;
    while (ros::ok())
    {
        if ( verbose == true )
        {
            ROS_INFO("Waiting for pose data");
        }
        if ( waiting_counter_pose >= waiting_threshold )
        {
            if ( verbose == true )
            {
                ROS_INFO("Waiting too long - trying to reconnect to   %s", connection_info_image.c_str());
                ROS_INFO("Waiting too long - trying to reconnect to   %s", connection_info_pose.c_str());
            }
            // Signalize within ROS, that we are still alive
            std_msgs::Bool msg;
            msg.data = true;
            publisher_heartbeat.publish(msg);
            // try to reconnect
            airsim_to_ros_image.Connect(connection_info_image);
            airsim_to_ros_pose.Connect(connection_info_pose);
            // Reset both counters or there could be two heartbeats sent shortly after one another
            waiting_counter_image = 0;
            waiting_counter_pose  = 0;
        }
        waiting_counter_pose++;
        int8_t received_return_value_pose  = airsim_to_ros_pose.ReceivedMessage();
        if (1 == received_return_value_pose)
        {
            waiting_counter_pose = 0;
            if ( verbose == true )
            {
                ROS_INFO("Pose received");
            }
            // Header
            //airsim_pose_msg.header.seq         = airsim_to_ros_pose.GetImageHeaderSeq();
            //airsim_pose_msg.header.stamp.sec   = airsim_to_ros_pose.GetImageHeaderStampSec();
            //airsim_pose_msg.header.stamp.nsec  = airsim_to_ros_pose.GetImageHeaderStampNsec();
            //airsim_pose_msg.header.frame_id    = airsim_to_ros_pose.GetImageHeaderFrameid();
            // Pose
            airsim_pose_msg.position.x          = airsim_to_ros_pose.GetPosePositionX();
            airsim_pose_msg.position.y          = airsim_to_ros_pose.GetPosePositionY();
            airsim_pose_msg.position.z          = airsim_to_ros_pose.GetPosePositionZ();
            
            tf::Quaternion q = tf::createQuaternionFromRPY(airsim_to_ros_pose.GetPoseOrientationRoll(),
                                                           airsim_to_ros_pose.GetPoseOrientationPitch(),
                                                           airsim_to_ros_pose.GetPoseOrientationYaw());  // Create this quaternion from roll/pitch/yaw (in radians)
            airsim_pose_msg.orientation.x       = q[0];
            airsim_pose_msg.orientation.y       = q[1];
            airsim_pose_msg.orientation.z       = q[2];
            airsim_pose_msg.orientation.w       = q[3];
            
            if ( verbose == true )
            {
                ROS_INFO("Pose received");
                ROS_INFO("  Pose.header.seq             %d", airsim_to_ros_pose.GetPoseHeaderSeq());
                ROS_INFO("  Pose.header.stamp.sec       %d", airsim_to_ros_pose.GetPoseHeaderStampSec());
                ROS_INFO("  Pose.header.stamp.nsec      %d", airsim_to_ros_pose.GetPoseHeaderStampNsec());
                ROS_INFO("  Pose.header.frame_id        %s", airsim_to_ros_pose.GetPoseHeaderFrameid().c_str());
                ROS_INFO("  Pose.position.x             %f", airsim_to_ros_pose.GetPosePositionX());
                ROS_INFO("  Pose.position.y             %f", airsim_to_ros_pose.GetPosePositionY());
                ROS_INFO("  Pose.position.z             %f", airsim_to_ros_pose.GetPosePositionZ());
                ROS_INFO("  Pose.orientation.roll       %f", airsim_to_ros_pose.GetPoseOrientationRoll());
                ROS_INFO("  Pose.orientation.pitch      %f", airsim_to_ros_pose.GetPoseOrientationPitch());
                ROS_INFO("  Pose.orientation.yaw        %f", airsim_to_ros_pose.GetPoseOrientationYaw());
            }

            pose_chatter.publish(airsim_pose_msg);
        }
        else if (-1 == received_return_value_pose)
        {
            ROS_INFO("Invalid Pose received - did not forward");
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////////
        
        if ( verbose == true )
        {
            ROS_INFO("Waiting for image data");
        }
        if ( waiting_counter_image >= waiting_threshold )
        {
            if ( verbose == true )
            {
                ROS_INFO("Waiting too long - trying to reconnect to   %s", connection_info_image.c_str());
                ROS_INFO("Waiting too long - trying to reconnect to   %s", connection_info_pose.c_str());
            }
            // Signalize within ROS, that we are still alive
            std_msgs::Bool msg;
            msg.data = true;
            publisher_heartbeat.publish(msg);
            // try to reconnect
            airsim_to_ros_image.Connect(connection_info_image);
            airsim_to_ros_pose.Connect(connection_info_pose);
            // Reset both counters or there could be two heartbeats sent shortly after one another
            waiting_counter_image = 0;
            waiting_counter_pose  = 0;
        }
        waiting_counter_image++;
        int8_t received_return_value_image = airsim_to_ros_image.ReceivedMessage();
        if (1 == received_return_value_image)
        {
            waiting_counter_image = 0;
            switch (airsim_to_ros_image.GetImageType())
            {
            case 0: // Unknown
                break;
            case 1: // Left
                if ( verbose == true )
                {
                    ROS_INFO("Left image received");
                }
                // Header
                airsim_image_left_msg.header.seq         = airsim_to_ros_image.GetImageHeaderSeq();
                airsim_image_left_msg.header.stamp.sec   = airsim_to_ros_image.GetImageHeaderStampSec();
                airsim_image_left_msg.header.stamp.nsec  = airsim_to_ros_image.GetImageHeaderStampNsec();
                airsim_image_left_msg.header.frame_id    = airsim_to_ros_image.GetImageHeaderFrameid();
                // Image
                airsim_image_left_msg.height             = airsim_to_ros_image.GetImageHeight();
                airsim_image_left_msg.width              = airsim_to_ros_image.GetImageWidth();
                airsim_image_left_msg.encoding           = airsim_to_ros_image.GetImageEncoding();
                airsim_image_left_msg.is_bigendian       = airsim_to_ros_image.GetImageIsBigendian();
                airsim_image_left_msg.step               = airsim_to_ros_image.GetImageStep();
                airsim_image_left_msg.data.resize(airsim_to_ros_image.GetImageDataSize());
                memcpy((char*)(&airsim_image_left_msg.data[0]), airsim_to_ros_image.GetImageData(), airsim_to_ros_image.GetImageDataSize());
                break;
            case 2: // Right
                if ( verbose == true )
                {
                    ROS_INFO("Right image received");
                }
                // Header
                airsim_image_right_msg.header.seq         = airsim_to_ros_image.GetImageHeaderSeq();
                airsim_image_right_msg.header.stamp.sec   = airsim_to_ros_image.GetImageHeaderStampSec();
                airsim_image_right_msg.header.stamp.nsec  = airsim_to_ros_image.GetImageHeaderStampNsec();
                airsim_image_right_msg.header.frame_id    = airsim_to_ros_image.GetImageHeaderFrameid();
                // Image
                airsim_image_right_msg.height             = airsim_to_ros_image.GetImageHeight();
                airsim_image_right_msg.width              = airsim_to_ros_image.GetImageWidth();
                airsim_image_right_msg.encoding           = airsim_to_ros_image.GetImageEncoding();
                airsim_image_right_msg.is_bigendian       = airsim_to_ros_image.GetImageIsBigendian();
                airsim_image_right_msg.step               = airsim_to_ros_image.GetImageStep();
                airsim_image_right_msg.data.resize(airsim_to_ros_image.GetImageDataSize());
                memcpy((char*)(&airsim_image_right_msg.data[0]), airsim_to_ros_image.GetImageData(), airsim_to_ros_image.GetImageDataSize());
                break;
            default:
                break;
            }
            
            if ( verbose == true )
            {
                ROS_INFO("Image received");
                ROS_INFO("  Image.header.seq %d", airsim_to_ros_image.GetImageHeaderSeq());
                ROS_INFO("  Image.header.stamp.sec %d", airsim_to_ros_image.GetImageHeaderStampSec());
                ROS_INFO("  Image.header.stamp.nsec %d", airsim_to_ros_image.GetImageHeaderStampNsec());
                ROS_INFO("  Image.header.frame_id %s", airsim_to_ros_image.GetImageHeaderFrameid().c_str());
                ROS_INFO("  Image.height %d", airsim_to_ros_image.GetImageHeight());
                ROS_INFO("  Image.width %d", airsim_to_ros_image.GetImageWidth());
                ROS_INFO("  Image.encoding %s", airsim_to_ros_image.GetImageEncoding().c_str());
                ROS_INFO("  Image.is_bigendian %d", airsim_to_ros_image.GetImageIsBigendian());
                ROS_INFO("  Image.step %d", airsim_to_ros_image.GetImageStep());
                ROS_INFO("  size(Image.data) %d", static_cast<int>(airsim_to_ros_image.GetImageDataSize()));
            }
            
            if (
                    airsim_image_left_msg.header.seq == airsim_image_right_msg.header.seq 
                &&  airsim_image_left_msg.header.seq > last_sequence_sent
            )
            {
                if ( verbose == true )
                {
                    ROS_INFO("Images forwarded");
                }
                left_stereoimage_chatter.publish(airsim_image_left_msg);
                right_stereoimage_chatter.publish(airsim_image_right_msg);
            }
        }
        else if (-1 == received_return_value_image)
        {
            ROS_INFO("Invalid Image received - did not forward");
        }
    }

    return 0;
}
