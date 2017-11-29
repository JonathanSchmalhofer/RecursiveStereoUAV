///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to wrap airsim_to_ros to receive Data from AirSim and make available in ROS as ROS-Message
///
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros("tcp://192.168.178.56:5676");
    
    ROS_INFO("Created airsim_to_ros");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher left_stereoimage_chatter = image_transport.advertise("/AirSimLeftStereoImage", 1);
    image_transport::Publisher right_stereoimage_chatter = image_transport.advertise("/AirSimRightStereoImage", 1);

    while (ros::ok())
    {
        ROS_INFO("Waiting for data");
        int8_t received_return_value = airsim_to_ros.ReceivedMessage();
        if (1 == received_return_value)
        {
            // Fill ROS message with Flatbuffers message received via ZeroMq
            sensor_msgs::Image airsim_image_msg;
            
            // Header
            airsim_image_msg.header.seq         = airsim_to_ros.GetImageHeaderSeq();
            airsim_image_msg.header.stamp.sec   = airsim_to_ros.GetImageHeaderStampSec();
            airsim_image_msg.header.stamp.nsec  = airsim_to_ros.GetImageHeaderStampNsec();
            airsim_image_msg.header.frame_id    = airsim_to_ros.GetImageHeaderFrameid();
            // Image
            airsim_image_msg.height             = airsim_to_ros.GetImageHeight();
            airsim_image_msg.width              = airsim_to_ros.GetImageWidth();
            airsim_image_msg.encoding           = airsim_to_ros.GetImageEncoding();
            airsim_image_msg.is_bigendian       = airsim_to_ros.GetImageIsBigendian();
            airsim_image_msg.step               = airsim_to_ros.GetImageStep();
            airsim_image_msg.data.resize(airsim_to_ros.GetImageDataSize());
            memcpy((char*)(&airsim_image_msg.data[0]), airsim_to_ros.GetImageData(), airsim_to_ros.GetImageDataSize());

            
            // TODO: Introduce Debug Mode
            ROS_INFO("Image received");
            ROS_INFO("  Image.header.seq %d", airsim_to_ros.GetImageHeaderSeq());
            ROS_INFO("  Image.header.stamp.sec %d", airsim_to_ros.GetImageHeaderStampSec());
            ROS_INFO("  Image.header.stamp.nsec %d", airsim_to_ros.GetImageHeaderStampNsec());
            ROS_INFO("  Image.header.frame_id %s", airsim_to_ros.GetImageHeaderFrameid());
            ROS_INFO("  Image.height %d", airsim_to_ros.GetImageHeight());
            ROS_INFO("  Image.width %d", airsim_to_ros.GetImageWidth());
            ROS_INFO("  Image.encoding %s", airsim_to_ros.GetImageEncoding());
            ROS_INFO("  Image.is_bigendian %d", airsim_to_ros.GetImageIsBigendian());
            ROS_INFO("  Image.step %d", airsim_to_ros.GetImageStep());
            ROS_INFO("  size(Image.data) %d", airsim_to_ros.GetImageDataSize());
            
            
            switch (airsim_to_ros.GetImageType())
            {
            case 0: // Unknown
                break;
            case 1: // Left
                ROS_INFO("Left image forwarded");
                left_stereoimage_chatter.publish(airsim_image_msg);
                break;
            case 2: // Right
                ROS_INFO("Right image forwarded");
                right_stereoimage_chatter.publish(airsim_image_msg);
                break;
            default:
                break;
            }
        }
        else if (-1 == received_return_value)
        {
            ROS_INFO("Invalid Image received - did not forward");
        }
    }

    return 0;
}
