#include "js_trajectory_planning_node/rrtframe.h"

// ----------------------------------------------------------------------------
// RRTFrame: main application window
// ----------------------------------------------------------------------------

RRTFrame::RRTFrame()
    : wxFrame(NULL, wxID_ANY, wxT("wxWidgets OpenGL"))
{
    canvas_ = new RRTGLCanvas(this, NULL);


    SetClientSize(2*khalf_width, 2*khalf_height);
    Show();

    // test IsDisplaySupported() function:
    static const int attribs[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, 0 };
    wxLogStatus("Double-buffered display %s supported",
                wxGLCanvas::IsDisplaySupported(attribs) ? "is" : "not");
}

RRTGLCanvas* RRTFrame::GetCanvas()
{
    return canvas_;
}

void RRTFrame::OnClose(wxCommandEvent& WXUNUSED(event))
{
    // true is to force the frame to close
    Close(true);
}
