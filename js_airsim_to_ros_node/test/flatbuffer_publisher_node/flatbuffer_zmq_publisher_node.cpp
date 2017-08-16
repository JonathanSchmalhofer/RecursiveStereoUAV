#include <string>
#include "ros/ros.h"
#include "zeromq_cpp/zmq.hpp"
#include "StarBuffers_generated.h"
#include "Image_generated.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flatbuffer_zmq_publisher_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting node 'flatbuffer_zmq_publisher_node'");
    
    zmq::context_t context(1);

    //  We send updates via this socket
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB);
    publishSocket.bind("tcp://*:5565");
    
    sleep(5);
    
    ROS_INFO("Publisher created");

    flatbuffers::FlatBufferBuilder fbb;
    StarBufferBuilder builder(fbb);

    builder.add_radius(42.42);
    builder.add_mass(51966); // hex2dec('CAFE') = 51966
    builder.add_volume(3.141592653589793); // pi (from MATLAB)

    auto response = builder.Finish();
    fbb.Finish(response);

    ROS_INFO("   Sending Star");
    int buffersize = fbb.GetSize();
    zmq::message_t request(buffersize);
    memcpy((void *)request.data(), fbb.GetBufferPointer(), buffersize);
    publishSocket.send(request);
    ROS_INFO("   Star Sent");
    
    return 0;
}
