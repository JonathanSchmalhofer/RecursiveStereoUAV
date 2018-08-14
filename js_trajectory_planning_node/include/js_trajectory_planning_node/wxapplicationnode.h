#ifndef JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H
#define JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H

#include "planner_wrapper.h"
#include "rrtglcontext.h"
#include "rrtframe.h"
//#include <js_messages/PointCloud.h>
#include <sensor_msgs/PointCloud.h>
#include <js_messages/Trajectory3D.h>


////////////////////////////////////////////////////////////////////////////////////

class wxApplicationNode
    : public wxApp
{
public:
    wxApplicationNode() { context_ = NULL;}

    // Returns the shared context used by all frames and sets it as current for
    // the given canvas.
    RRTGLContext& GetContext(wxGLCanvas *canvas);

    // virtual wxApp methods
    virtual bool OnInit();
    virtual int OnExit();

    void UpdateDrawnCanvas(wxIdleEvent &event);
	void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& pointcloud_message);

    RRTFrame* GetFrame();
    PlannerWrapper* GetPlanner();

private:
    void ExtractStartAndGoalStateForContext();
    void ExtractPointsAndLinesFromTreeForContext();

    PlannerWrapper *planner_;
    // the GL context we use for all our mono rendering windows
    RRTGLContext *context_;
    RRTFrame *frame_;
	ros::NodeHandle node_handle_;
	ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    wxDECLARE_EVENT_TABLE();
};

#endif // JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H
