#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"

void mapFlatbuffersImageToRosImage(const airsim_to_ros::Image* flatbuffers_image, sensor_msgs::ImagePtr& ros_image)
{
    // Header
    ros_image->header.seq = flatbuffers_image->header()->seq();
    ros_image->header.stamp.sec = flatbuffers_image->header()->stamp()->sec();
    ros_image->header.stamp.nsec = flatbuffers_image->header()->stamp()->nsec();
    ros_image->header.frame_id = flatbuffers_image->header()->frame_id()->c_str();
    // Image
    ros_image->height = flatbuffers_image->height();
    ros_image->width = flatbuffers_image->width();
    ros_image->encoding = flatbuffers_image->encoding()->c_str();
    ros_image->is_bigendian = flatbuffers_image->is_bigendian();
    ros_image->step = flatbuffers_image->step();
    size_t size = flatbuffers_image->step() * flatbuffers_image->height();
    ros_image->data.resize(size);
    memcpy((char*)(&ros_image->data[0]), flatbuffers_image->data(), size);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros("tcp://localhost:5676");
    
    ROS_INFO("Created airsim_to_ros");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher chatterAirSimMessage = image_transport.advertise("/AirSimImage", 1);

    while (ros::ok())
    {
        ROS_INFO("Waiting for data");
        if (airsim_to_ros.ReceivedMessage())
        {
            // Reconstruct image from Flatbuffers received
            flatbuffers::FlatBufferBuilder fbb;
            airsim_to_ros::ImageBuilder builder(fbb);
            auto fb_image_rcvd = airsim_to_ros::GetImage(airsim_to_ros.GetReceivedMessageData());
            
            // Fill ROS message with Flatbuffers message received via ZeroMq
            sensor_msgs::ImagePtr airsim_image_msg;
            mapFlatbuffersImageToRosImage(fb_image_rcvd, airsim_image_msg);
            chatterAirSimMessage.publish(airsim_image_msg);
            ROS_INFO("Image forwarded");
        }
    }

    return 0;
}
