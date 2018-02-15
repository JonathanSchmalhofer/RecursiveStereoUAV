#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTFRAME_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTFRAME_H

#include "rrtglcanvas.h"

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

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTFRAME_H
