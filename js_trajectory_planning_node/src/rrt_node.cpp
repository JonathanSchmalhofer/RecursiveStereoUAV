#include "js_trajectory_planning_node/rrt_node.h"
#include <sensor_msgs/PointCloud.h>
#include <js_messages/Trajectory3D.h>
#include <js_common/common.h>

////////////////////////////////////////////////////////////////////////////////////

// Using a global variable to trigger methods within the wxWidget
// Somehow I cannot find a better way to integrate ROS with wxWidgets
// The ones described here do not seem to work for me: https://answers.ros.org/question/11936/communication-between-ros-and-wxwidgets/
bool external_trigger_subscriber_pcl  = false;        // input
bool external_trigger_subscriber_pose = false;        // input
bool external_trigger_publisher       = false;        // output
sensor_msgs::PointCloud current_point_cloud;     // input
geometry_msgs::Pose current_pose;                // input
js_messages::Trajectory3D current_trajectory_3d; // output

double goal_x, goal_y, goal_z;
double size_x, size_y, size_z;

ros::Subscriber subscriber_pcl_;
ros::Subscriber subscriber_pose_;
ros::Publisher publisher_;

void ExternalTriggerSubscriberPcl(const sensor_msgs::PointCloudConstPtr& pointcloud_message)
{
    // Todo: can be optimized!!!
    // Store
    //   see: http://library.isr.ist.utl.pt/docs/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Sensors.html
    //        http://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
    current_point_cloud.points.clear();
    current_point_cloud.points.resize(pointcloud_message->points.size());
    for (int i = 0; i < pointcloud_message->points.size(); i++)
    {
        current_point_cloud.points[i].x = pointcloud_message->points.at(i).x;
        current_point_cloud.points[i].y = pointcloud_message->points.at(i).y;
        current_point_cloud.points[i].z = pointcloud_message->points.at(i).z;
    }
    external_trigger_subscriber_pcl = true; // Setting to true will make the wxApplication call PointCloudCallback()
}

void ExternalTriggerSubscriberPose(const geometry_msgs::Pose::ConstPtr& pose_message)
{
    current_pose.position.x = pose_message->position.x;
    current_pose.position.y = pose_message->position.y;
    current_pose.position.z = pose_message->position.z;
    current_pose.orientation.x = pose_message->orientation.x;
    current_pose.orientation.y = pose_message->orientation.y;
    current_pose.orientation.z = pose_message->orientation.z;
    current_pose.orientation.w = pose_message->orientation.w;
    
    external_trigger_subscriber_pose = true; // Setting to true will make the wxApplication call PoseCallback()
}

void ExternalTriggerPublisher()
{
    if(external_trigger_publisher)  // will be true once PointCloudCallback() was called
    {
        external_trigger_publisher = false;
        publisher_.publish(current_trajectory_3d);
    }
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle_;
    
    // Setup Publisher and Subscriber
    subscriber_pose_ = node_handle_.subscribe("/airsim/pose", 1,       ExternalTriggerSubscriberPose);
    subscriber_pcl_  = node_handle_.subscribe("/airsim/pointcloud", 1, ExternalTriggerSubscriberPcl);
    publisher_       = node_handle_.advertise<js_messages::Trajectory3D>("/trajectory_planning/trajectory3d", 1);

    ROS_INFO("Starting RRT 3d node with wxWidget visualization");
    
    // Get Parameters from Launchfile
    js_common::TryGetParameter("/trajectory_planning/parameters/goal_x", goal_x, 400.0f);
    js_common::TryGetParameter("/trajectory_planning/parameters/goal_y", goal_y,   0.0f);
    js_common::TryGetParameter("/trajectory_planning/parameters/goal_z", goal_z,  10.0f);
    js_common::TryGetParameter("/trajectory_planning/parameters/size_x", size_x, 800.0f);
    js_common::TryGetParameter("/trajectory_planning/parameters/size_y", size_y, 800.0f);
    js_common::TryGetParameter("/trajectory_planning/parameters/size_z", size_z,  50.0f);

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see UpdateDrawnCanvas() and PointCloudCallback()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
