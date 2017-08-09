///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief AirSimToRos node initialization
///

#include "airsimros_airsim_to_ros/airsim_to_ros.h"

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "airsimros_airsim_to_ros_converter");
        ros::NodeHandle node_handle;

        std::string ip_adress_simulator = "127.0.0.1";
        std::string port_simulator = "30819";
        std::string out_stereo_image_topic_name = "stereo_image";


        //-----------------Receive osi via zmq, copy data to ROS msg and publish it
        ros::Publisher publisherStereoImage =
            node_handle.advertise<uint8_t>(out_stereo_image_topic_name, 1000);
			
        while (ros::ok())
        {
            uint8_t airsimros_stereo_image_message = 0;

            chatterGroundtruth.publish(airsimros_stereo_image_message);
        }
    }
    catch (const ros::InvalidNodeNameException& e)
    {
        ROS_ERROR("Node name invalid: %s", e.what());
        return -1;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error: %s", e.what());
        return -1;
    }

    return 0;
}
