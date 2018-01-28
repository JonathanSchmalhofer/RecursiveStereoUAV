///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief Node to open a wxWidget GUI.
///
#include <string>
#include <ros/ros.h>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

class BasicDrawPane
    : public wxPanel
{
 
public:
    BasicDrawPane(wxFrame* parent);
 
    void paintEvent(wxPaintEvent & evt);
    void paintNow();
 
    void render(wxDC& dc);
 
private:
    wxDECLARE_EVENT_TABLE();
};

class wxApplicationNode
    : public wxApp
{
public:
    virtual bool OnInit();
    void DoUpdate(wxIdleEvent &event);
 
private:
    wxFrame *frame;
    BasicDrawPane *drawPane;
 
    wxDECLARE_EVENT_TABLE();
};

BasicDrawPane::BasicDrawPane(wxFrame* parent)
    : wxPanel(parent)
{
}
 
void BasicDrawPane::paintEvent(wxPaintEvent & evt)
{
    wxPaintDC dc(this);
    render(dc);
}
 
void BasicDrawPane::paintNow()
{
    wxClientDC dc(this);
    render(dc);
}

void BasicDrawPane::render(wxDC&  dc)
{ 
    //  node drawing radius
    const double r = 1;
    
    int p1_x = 200;
    int p1_y = 100;
    int p2_x = 220;
    int p2_y = 108;
    
    // circles
    dc.SetBrush(*wxGREEN_BRUSH); // green filling
    dc.SetPen( wxPen( wxColor(255,0,0), 2 ) ); // 2-pixels-thick red outline
    
    // draw a circle around p1 and p2
    dc.DrawCircle( wxPoint(p1_x,p1_y), r /* radius */ );
    dc.DrawCircle( wxPoint(p2_x,p2_y), r /* radius */ );
 
    // draw a line from p1 to p2
    dc.SetPen( wxPen( wxColor(0,0,0), 1 ) ); // black line, 3 pixels thick
    dc.DrawLine( p1_x, p1_y, p2_x, p2_y ); // draw line across the rectangle
}

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()

wxBEGIN_EVENT_TABLE(BasicDrawPane, wxPanel)
    EVT_PAINT(BasicDrawPane::paintEvent)
wxEND_EVENT_TABLE()

IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

bool wxApplicationNode::OnInit()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
    frame = new wxFrame((wxFrame *)NULL, -1,  wxT("wxWidget node"), wxPoint(50,50), wxSize(800,600));
    
    drawPane = new BasicDrawPane( (wxFrame*) frame );
    sizer->Add(drawPane, 1, wxEXPAND);
 
    frame->SetSizer(sizer);
    frame->SetAutoLayout(true);
 
    frame->Show(true);
    return true;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{
    if(ros::ok())
    {
        ROS_INFO("SpinOnce()");
        ros::spinOnce();
	    ros::Duration(1).sleep();
    }

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "wxwidget_node");
    
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting wxWidget node"); 

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}

