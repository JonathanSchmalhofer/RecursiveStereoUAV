#include "js_trajectory_planning_node/rrt_node.h"
#include <sensor_msgs/PointCloud.h>
#include <js_messages/Trajectory3D.h>

////////////////////////////////////////////////////////////////////////////////////

// Using a global variable to trigger methods within the wxWidget
// Somehow I cannot find a better way to integrate ROS with wxWidgets
// The ones described here do not seem to work for me: https://answers.ros.org/question/11936/communication-between-ros-and-wxwidgets/
bool external_trigger_subscriber = false;        // input
bool external_trigger_publisher  = false;        // output
sensor_msgs::PointCloud current_point_cloud;     // input
js_messages::Trajectory3D current_trajectory_3d; // output

ros::Subscriber subscriber_;
ros::Publisher publisher_;

void ExternalTriggerSubscriber(const sensor_msgs::PointCloudConstPtr& pointcloud_message)
{
    // Todo: can be optimized!!!
    // Store PointCloud
    current_point_cloud.points.clear();
    for (int i = 0; i < pointcloud_message->points.size(); i++)
    {
        geometry_msgs::Point32 new_point(pointcloud_message->points.at(i).x,
                                         pointcloud_message->points.at(i).y,
                                         pointcloud_message->points.at(i).z);
        current_point_cloud.points.push_back(new_point);
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    
	external_trigger_subscriber = true; // Setting to true will make the wxApplication call PointCloudCallback()
    do
    {
        if(external_trigger_publisher)  // will be true once PointCloudCallback() was called
        {
            external_trigger_publisher = false;
            ROS_INFO("GO PUBLISH");
        }
    }
    while(external_trigger_subscriber); // will be false until we published the Trajectory
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle_;
	
	// Setup Publisher and Subscriber
    subscriber_ = node_handle_.subscribe("/airsim/pointcloud", 1, ExternalTriggerSubscriber);
    publisher_  = node_handle_.advertise<js_messages::Trajectory3D>("/trajectory_planning/trajectory3d", 1);

    ROS_INFO("Starting RRT 3d node with wxWidget visualization");

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see UpdateDrawnCanvas() and PointCloudCallback()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
