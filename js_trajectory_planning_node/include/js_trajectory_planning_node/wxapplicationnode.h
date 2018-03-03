#ifndef JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H
#define JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H

#include "planner_wrapper.h"
#include "rrtglcontext.h"
#include "rrtframe.h"


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

    void DoUpdate(wxIdleEvent &event);

    RRTFrame* GetFrame();
    PlannerWrapper* GetPlanner();

private:
    void ExtractStartAndGoalStateForContext();
    void ExtractPointsAndLinesFromTreeForContext();

    PlannerWrapper *planner_;
    // the GL context we use for all our mono rendering windows
    RRTGLContext *context_;
    RRTFrame *frame_;

    wxDECLARE_EVENT_TABLE();
};

#endif // JS_TRAJECTORY_PLANNING_NODE_WXAPPLICATIONNODE_H
