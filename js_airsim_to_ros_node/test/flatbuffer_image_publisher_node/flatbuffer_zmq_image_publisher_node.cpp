#include <string>
#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
    /*
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/q386212/master_ws/src/RecursiveStereoUAV/js_airsim_to_ros_node/test/flatbuffer_image_publisher_node/lena.png", CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/AirSimImage", 1);
    ros::Rate loop_rate(5);

    while (nh.ok()) 
    {
        pub.publish(ros_image);
        loop_rate.sleep();
    }
    */
    ros::init(argc, argv, "flatbuffer_zmq_image_publisher_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting node 'flatbuffer_zmq_image_publisher_node'");
    
    zmq::context_t context(1);

    //  We send updates via this socket
    zmq::socket_t publishSocket = zmq::socket_t(context, ZMQ_PUB);
    publishSocket.bind("tcp://*:5676");
    
    sleep(5);
    
    ROS_INFO("Image Publisher created");
    
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/q386212/master_ws/src/RecursiveStereoUAV/js_airsim_to_ros_node/test/flatbuffer_image_publisher_node/lena.png", CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "rgb8";
    
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    flatbuffers::FlatBufferBuilder fbb;
    // Image.header.stamp
    airsim_to_ros::time message_time(ros_image.header.stamp.sec,ros_image.header.stamp.nsec);
    // Image.header
    auto header = airsim_to_ros::CreateHeader(
        fbb, 
        ros_image.header.seq,
        &message_time,
        fbb.CreateString(ros_image.header.frame_id));
    fbb.Finish(header);
    // Image
    std::vector<std::uint8_t> image_data(3 * 640 * 480);
    auto image = airsim_to_ros::CreateImage(
        fbb, 
        header, 
        ros_image.height,
        ros_image.width,
        fbb.CreateString(ros_image.encoding),
        ros_image.is_bigendian,
        ros_image.step,
        fbb.CreateVector<std::uint8_t>(ros_image.data));
    fbb.Finish(image);

    ROS_INFO("   Sending Image");
    int buffersize = fbb.GetSize();
    zmq::message_t image_msg(buffersize);
    memcpy((void *)image_msg.data(), fbb.GetBufferPointer(), buffersize);
    publishSocket.send(image_msg);
    ROS_INFO("   Image Sent (%d Bytes)", buffersize);
    
    sleep(5);
    
    return 0;
}
