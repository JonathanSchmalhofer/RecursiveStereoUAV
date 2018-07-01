#include "js_trajectory_planning_node/rrt_node.h"
#include <js_messages/PointCloud.h>

////////////////////////////////////////////////////////////////////////////////////

void PointCloudCallback(const js_messages::PointCloud::ConstPtr& pointcloud_message)
{
    ROS_INFO("The sequence is: [%d]", pointcloud_message->header.seq);
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle;
    
    ros::Subscriber pointcloud_subscriber = node_handle.subscribe("/stereo_vision/pointcloud", 1000, PointCloudCallback);
    
    //ROS_INFO("Starting RRT 3d node with wxWidget visualization");

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see DoUpdate()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
