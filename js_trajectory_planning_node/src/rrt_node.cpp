#include "js_trajectory_planning_node/rrt_node.h"
#include <sensor_msgs/PointCloud.h>
#include <js_messages/Trajectory3D.h>

////////////////////////////////////////////////////////////////////////////////////

// Using a global variable to trigger methods within the wxWidget
// Somehow I cannot find a better way to integrate ROS with wxWidgets
// The ones described here do not seem to work for me: https://answers.ros.org/question/11936/communication-between-ros-and-wxwidgets/
bool external_trigger_subscriber = false;
bool external_trigger_publisher  = false;

ros::Subscriber subscriber_;
ros::Publisher publisher_;

void ExternalTriggerSubscriber(const sensor_msgs::PointCloudConstPtr& pointcloud_message)
{
	external_trigger_subscriber = true;
	if(external_trigger_publisher)
	{
		external_trigger_publisher = false;
		ROS_INFO("GO PUBLISH");
	}
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
