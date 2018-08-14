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
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

sensor_msgs::Image airsim_image_left_msg, airsim_image_right_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting AirSim to ros node");
    
    // Get Parameters from ROS Parameter Server
    bool verbose;
    std::string ip_airsim;
    std::string port_airsim_to_ros;
    js_common::TryGetParameter("/recursivestereo/parameters/verbose",            verbose,            false);
    js_common::TryGetParameter("/recursivestereo/parameters/ip_airsim",          ip_airsim,          "127.0.0.1");
    js_common::TryGetParameter("/recursivestereo/parameters/port_airsim_to_ros", port_airsim_to_ros, "5676");
    
    // Subscribe to IP of Host running AirSim
	std::string connection_info = "tcp://" + ip_airsim + ":" + port_airsim_to_ros;
	ROS_INFO("Subscribing to:    %s", connection_info.c_str());
    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros(connection_info);
    
    ROS_INFO("Created airsim_to_ros");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher left_stereoimage_chatter  = image_transport.advertise("/airsim/left/image_raw", 1);
    image_transport::Publisher right_stereoimage_chatter = image_transport.advertise("/airsim/right/image_raw", 1);
	ros::Publisher publisher_heartbeat                   = node_handle.advertise<std_msgs::Bool>("/airsim/heartbeat", 1);

    std::uint32_t last_sequence_sent = 0;
	std::uint32_t waiting_counter    = 0;
	std::uint32_t waiting_threshold  = 10;
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
			waiting_counter = 0;
        }
		waiting_counter++;
        int8_t received_return_value = airsim_to_ros.ReceivedMessage();
        if (1 == received_return_value)
        {
			waiting_counter = 0;
            switch (airsim_to_ros.GetImageType())
            {
            case 0: // Unknown
                break;
            case 1: // Left
                if ( verbose == true )
                {
                    ROS_INFO("Left image received");
                }
                // Header
                airsim_image_left_msg.header.seq         = airsim_to_ros.GetImageHeaderSeq();
                airsim_image_left_msg.header.stamp.sec   = airsim_to_ros.GetImageHeaderStampSec();
                airsim_image_left_msg.header.stamp.nsec  = airsim_to_ros.GetImageHeaderStampNsec();
                airsim_image_left_msg.header.frame_id    = airsim_to_ros.GetImageHeaderFrameid();
                // Image
                airsim_image_left_msg.height             = airsim_to_ros.GetImageHeight();
                airsim_image_left_msg.width              = airsim_to_ros.GetImageWidth();
                airsim_image_left_msg.encoding           = airsim_to_ros.GetImageEncoding();
                airsim_image_left_msg.is_bigendian       = airsim_to_ros.GetImageIsBigendian();
                airsim_image_left_msg.step               = airsim_to_ros.GetImageStep();
                airsim_image_left_msg.data.resize(airsim_to_ros.GetImageDataSize());
                memcpy((char*)(&airsim_image_left_msg.data[0]), airsim_to_ros.GetImageData(), airsim_to_ros.GetImageDataSize());
                break;
            case 2: // Right
                if ( verbose == true )
                {
                    ROS_INFO("Right image received");
                }
                // Header
                airsim_image_right_msg.header.seq         = airsim_to_ros.GetImageHeaderSeq();
                airsim_image_right_msg.header.stamp.sec   = airsim_to_ros.GetImageHeaderStampSec();
                airsim_image_right_msg.header.stamp.nsec  = airsim_to_ros.GetImageHeaderStampNsec();
                airsim_image_right_msg.header.frame_id    = airsim_to_ros.GetImageHeaderFrameid();
                // Image
                airsim_image_right_msg.height             = airsim_to_ros.GetImageHeight();
                airsim_image_right_msg.width              = airsim_to_ros.GetImageWidth();
                airsim_image_right_msg.encoding           = airsim_to_ros.GetImageEncoding();
                airsim_image_right_msg.is_bigendian       = airsim_to_ros.GetImageIsBigendian();
                airsim_image_right_msg.step               = airsim_to_ros.GetImageStep();
                airsim_image_right_msg.data.resize(airsim_to_ros.GetImageDataSize());
                memcpy((char*)(&airsim_image_right_msg.data[0]), airsim_to_ros.GetImageData(), airsim_to_ros.GetImageDataSize());
                break;
            default:
                break;
            }
            
            if ( verbose == true )
            {
                ROS_INFO("Image received");
                ROS_INFO("  Image.header.seq %d", airsim_to_ros.GetImageHeaderSeq());
                ROS_INFO("  Image.header.stamp.sec %d", airsim_to_ros.GetImageHeaderStampSec());
                ROS_INFO("  Image.header.stamp.nsec %d", airsim_to_ros.GetImageHeaderStampNsec());
                ROS_INFO("  Image.header.frame_id %s", airsim_to_ros.GetImageHeaderFrameid().c_str());
                ROS_INFO("  Image.height %d", airsim_to_ros.GetImageHeight());
                ROS_INFO("  Image.width %d", airsim_to_ros.GetImageWidth());
                ROS_INFO("  Image.encoding %s", airsim_to_ros.GetImageEncoding().c_str());
                ROS_INFO("  Image.is_bigendian %d", airsim_to_ros.GetImageIsBigendian());
                ROS_INFO("  Image.step %d", airsim_to_ros.GetImageStep());
                ROS_INFO("  size(Image.data) %d", static_cast<int>(airsim_to_ros.GetImageDataSize()));
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
        else if (-1 == received_return_value)
        {
            ROS_INFO("Invalid Image received - did not forward");
        }
    }

    return 0;
}
