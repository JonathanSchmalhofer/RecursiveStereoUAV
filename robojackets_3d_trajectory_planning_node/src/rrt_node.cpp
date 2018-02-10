#include "rrt_node.hpp"

////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle;
    
    //ROS_INFO("Starting RRT node with wxWidget visualization");

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see DoUpdate()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
