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
    ros::init(argc, argv, "flatbuffer_zmq_publisher_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting node 'flatbuffer_zmq_publisher_node'");
    
    zmq::context_t context(1);

    //  We send updates via this socket
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB);
    publishSocket.bind("tcp://*:5676");
    
    sleep(5);
    
    ROS_INFO("Publisher created");

    //std::cout << "Press enter to continue ...";
    //std::cin.get();

    flatbuffers::FlatBufferBuilder fbb;
    // Image.header.stamp
    airsim_to_ros::time message_time(42,24);
    // Image.header
    auto header = airsim_to_ros::CreateHeader(
        fbb, 
        21, 
        &message_time,
        fbb.CreateString("test-header-ID"));
    fbb.Finish(header);
    // Image
    std::vector<std::uint8_t> image_data(3 * 640 * 480);
    auto image = airsim_to_ros::CreateImage(
        fbb, 
        header, 
        480, 
        640, 
        fbb.CreateString("rgb8"), 
        isBigEndian(), 
        sizeof(std::uint8_t) * 3 * 640, 
        fbb.CreateVector<std::uint8_t>(image_data));
    fbb.Finish(image);

    ROS_INFO("   Sending Image");
    int buffersize = fbb.GetSize();
    zmq::message_t image_msg(buffersize);
    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffersize);
    publishSocket.send(image_msg);
    ROS_INFO("   Image Sent");
    
    sleep(5);
    
    return 0;
}
