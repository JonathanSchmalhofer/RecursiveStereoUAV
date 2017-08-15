#include <string>
#include "ros/ros.h"
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"
#include "zeromq_cpp/zmq.hpp"

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


    ros::Rate r(5);

    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
