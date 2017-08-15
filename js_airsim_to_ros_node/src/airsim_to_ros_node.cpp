#include <string>
#include "ros/ros.h"
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"
#include "zeromq_cpp/zmq.hpp"
#include "StarBuffers_generated.h"

struct Star {
    double radius ;
    double mass;
    double volume;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle nh;

    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros;
    
    zmq::context_t context(1);

    //  Connect our subscriber socket
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.setsockopt(ZMQ_IDENTITY, "Hello", 5);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    subscriber.connect("tcp://localhost:5565");
    
    zmq::message_t receiveMessage;
    subscriber.recv(&receiveMessage);

    flatbuffers::FlatBufferBuilder fbb;
    StarBufferBuilder builder(fbb);

    auto star = GetStarBuffer(receiveMessage.data());

    std::cout << "Received Star" << std::endl;
    std::cout << "radius: " << star->radius() << std::endl;
    std::cout << "mass: " << star->mass() << std::endl;
    std::cout << "volume: " << star->volume() << std::endl;


    ros::Rate r(5);

    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
