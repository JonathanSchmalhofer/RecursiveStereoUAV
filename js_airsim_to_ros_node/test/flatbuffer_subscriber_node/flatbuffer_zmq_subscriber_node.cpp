///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Example ros node to subscribe to flatbuffer messages sent via zmq
///
#include <string>
#include "ros/ros.h"
#include "zeromq_cpp/zmq.hpp"
#include "js_airsim_to_ros_library/Image_generated.h"

#include <iostream> // for std::cin.get();

std::uint8_t isBigEndian()
{
    // Source: https://stackoverflow.com/questions/280162/is-there-a-way-to-do-a-c-style-compile-time-assertion-to-determine-machines-e
    union {
        long int l;
        char c[sizeof (long int)];
    } u;

    u.l = 1;

    if (u.c[sizeof(long int)-1] == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flatbuffer_zmq_subscriber_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting node 'flatbuffer_zmq_subscriber_node'");
    
    zmq::context_t context(1);

    //  We send updates via this socket
    zmq::socket_t subscribeSocket = zmq::socket_t(context, ZMQ_SUB);
	subscribeSocket.setsockopt(ZMQ_IDENTITY, "flatbuffer_zmq_subscriber_node", 5);
    subscribeSocket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    subscribeSocket.connect("tcp://localhost:5676");
    
    ROS_INFO("Subscriber created");

    //std::cout << "Press enter to continue ...";
    //std::cin.get();
	
	while (1)
	{
        zmq::message_t receiveMessage;
        subscribeSocket.recv(&receiveMessage);
        
        flatbuffers::FlatBufferBuilder fbb;
        airsim_to_ros::ImageBuilder builder(fbb);
        
        // Verify the message contents are trustworthy.
        flatbuffers::Verifier verifier((const unsigned char *)receiveMessage.data(),
                                                              receiveMessage.size());
        
        auto fb_image_rcvd = airsim_to_ros::GetImage(receiveMessage.data());
        
        // Make sure the message has valid data.
        if (airsim_to_ros::VerifyImageBuffer(verifier))
        {
            std::cout << "Received Image"           << std::endl;
            std::cout << "   header.seq: "          <<  fb_image_rcvd->header()->seq()              << std::endl;
            std::cout << "   header.stamp.sec: "    <<  fb_image_rcvd->header()->stamp()->sec()     << std::endl;
            std::cout << "   header.stamp.nsec: "   <<  fb_image_rcvd->header()->stamp()->nsec()    << std::endl;    
        }
        else
        {
            std::cout << "Invalid Image !!!!"           << std::endl;
        }
	}
    return 0;
}
