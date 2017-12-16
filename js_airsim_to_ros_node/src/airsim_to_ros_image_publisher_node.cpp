///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to cyclically publish buffered PNG images
///
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/filesystem.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>

// Source for GetFileList(): https://gist.github.com/vivithemage/9517678
std::vector<std::string> GetFileList(const std::string& path)
{
    std::vector<std::string> file_list;
    if (!path.empty())
    {
        boost::filesystem::path apk_path(path);
        boost::filesystem::recursive_directory_iterator end;

        for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const boost::filesystem::path cp = (*i);
            if (".png" == boost::filesystem::extension(cp.string()))
            {
                file_list.push_back(cp.string());
            }
        }
    }
    std::sort(file_list.begin(), file_list.end());
    return file_list;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_to_ros_image_publisher_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting node");

    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher left_stereoimage_chatter = image_transport.advertise("/airsim/left/image_raw", 1);
    image_transport::Publisher right_stereoimage_chatter = image_transport.advertise("/airsim/right/image_raw", 1);

    std::vector<std::string> left_file_list = GetFileList("/home/johnny/camera_calibration/left");
    std::vector<std::string> right_file_list = GetFileList("/home/johnny/camera_calibration/right");
    
    std::vector<sensor_msgs::Image> left_image_messages;
    std::vector<sensor_msgs::Image> right_image_messages;
    
    // Buffer left images
    for (auto image_path : left_file_list)
    {        
        ROS_INFO("Reading left file %s: ", image_path.c_str());
        
        cv_bridge::CvImage cv_image;
        cv_image.image = cv::imread(image_path.c_str(), CV_LOAD_IMAGE_COLOR);
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        left_image_messages.push_back(ros_image);
    }
    
    // Buffer right images
    for (auto image_path : right_file_list)
    {
        ROS_INFO("Reading right file %s: ", image_path.c_str());
        
        cv_bridge::CvImage cv_image;
        cv_image.image = cv::imread(image_path.c_str(), CV_LOAD_IMAGE_COLOR);
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        right_image_messages.push_back(ros_image);
    }
    
    ros::Rate r(5.0); // 5.0 hz
    std::int32_t loop_counter = 0;
    std::int32_t loop_direction = +1;
    while (ros::ok())
    {
        if (loop_counter >= std::min(left_image_messages.size(), right_image_messages.size())-1)
        {
            ROS_INFO("Reverse direction");
            loop_direction = -1;
        }
        if (loop_counter <= 0)
        {
            ROS_INFO("Forward direction");
            loop_direction = +1;
        }
        
        ROS_INFO("Publishing %d: ", loop_counter);
        
        left_stereoimage_chatter.publish(left_image_messages.at(loop_counter));
        right_stereoimage_chatter.publish(right_image_messages.at(loop_counter));
        loop_counter += loop_direction;
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
