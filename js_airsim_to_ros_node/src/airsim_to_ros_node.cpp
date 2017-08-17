#include <string>
#include "ros/ros.h"
#include "js_airsim_to_ros_library/airsim_to_ros_class.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting node");

    js_airsim_to_ros_library::AirSimToRosClass airsim_to_ros;
    
    ROS_INFO("Created airsim_to_ros");

    ros::Rate r(5);

    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
        ROS_INFO("running");
    }

    return 0;
}
