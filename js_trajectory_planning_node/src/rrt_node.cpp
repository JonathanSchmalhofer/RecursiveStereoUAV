#include "js_trajectory_planning_node/rrt_node.h"

////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");

    ROS_INFO("Starting RRT 3d node with wxWidget visualization");

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see UpdateDrawnCanvas() and PointCloudCallback()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
