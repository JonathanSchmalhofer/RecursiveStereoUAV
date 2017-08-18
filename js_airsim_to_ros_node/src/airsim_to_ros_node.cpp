#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros;
    
    ROS_INFO("Created airsim_to_ros");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher chatterAirSimMessage = image_transport.advertise("/AirSimImage", 1);

    while (ros::ok())
    {
        if (airsim_to_ros.ReceivedMessage())
        {
            // Todo: fill ROS-Message with Zmq-Message
            sensor_msgs::ImagePtr airsim_image_msg;
            if (true)
            {
                chatterAirSimMessage.publish(airsim_image_msg);
                ROS_INFO("Image sent");
            }
        }
    }

    return 0;
}
