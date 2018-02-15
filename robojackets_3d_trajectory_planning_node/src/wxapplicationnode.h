#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H

#include "planner_wrapper.h"
#include "rrtglcontext.h"
#include "rrtglcanvas.h"


// ----------------------------------------------------------------------------
// helper functions
// ----------------------------------------------------------------------------

const int khalf_width = 400;
const int khalf_height = 400;
const int khalf_depth = 400;

const double kdelta_angle = 5; // [deg]
const double kdelta_translation = 10; // [-]

static void CheckGLError();

////////////////////////////////////////////////////////////////////////////////////



// Define a new frame type
class RRTFrame : public wxFrame
{
public:
    RRTFrame();

    RRTGLCanvas* GetCanvas();

private:
    RRTGLCanvas* canvas_;

    void OnClose(wxCommandEvent& event);

    wxDECLARE_EVENT_TABLE();
};



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

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H
