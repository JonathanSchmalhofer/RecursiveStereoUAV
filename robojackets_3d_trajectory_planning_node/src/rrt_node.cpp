#include <string>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory> // for make_unique
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "2dplane/2dplane.hpp"
#include "planning/Path.hpp"

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include <wx/glcanvas.h>
 
#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
 
#ifndef WIN32
#include <unistd.h> // FIXME: ¿This work/necessary in Windows?
                    //Not necessary, but if it was, it needs to be replaced by process.h AND io.h
#endif


class PlannerWrapper
{
public:
    PlannerWrapper();
    void step();

    std::shared_ptr<RRT::GridStateSpace> _stateSpace;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector2d>> _biRRT;

    Eigen::Vector2d _startVel;
    Eigen::Vector2d _goalVel;
    std::vector<Eigen::Vector2d> _previousSolution;
};

PlannerWrapper::PlannerWrapper()
{
    Eigen::Vector2d size(800, 600);
    _stateSpace = std::make_shared<RRT::GridStateSpace>(size.x(), size.y(), 40, 30);
    _biRRT = std::make_unique<RRT::BiRRT<Eigen::Vector2d>>(_stateSpace, 2);

    //  setup birrt
    _biRRT->setStartState(size / 10);
    _biRRT->setGoalState(size / 2);
    _biRRT->setMaxStepSize(100);
    _biRRT->setStepSize(10);
    _biRRT->setMaxIterations(1000);
    _biRRT->setASCEnabled(true);
    _biRRT->setGoalBias(0.2);
    _biRRT->setWaypointBias(0.2);
    _biRRT->setGoalMaxDist(5);
    

    _startVel = Eigen::Vector2d(3, 0);
    _goalVel = Eigen::Vector2d(0, 3);
}

void PlannerWrapper::step()
{
    int numTimes = 5;

    for (int i = 0; i < numTimes; i++)
    {
        _biRRT->grow();
    }

    ROS_INFO("#%d with size() = %d", _biRRT->iterationCount(), _biRRT->getPath().size());

    // store solution
    _previousSolution.clear();
    if (_biRRT->startSolutionNode() != nullptr)
    {
        ROS_INFO("   Found a solution");
        _previousSolution = _biRRT->getPath();
        RRT::SmoothPath(_previousSolution, *_stateSpace);
    }
}

////////////////////////////////////////////////////////////////////////////////////

class wxGLCanvasSubClass
    : public wxGLCanvas
{
public:
    wxGLCanvasSubClass(wxFrame* parent);
    void Render();
    void Paintit(wxPaintEvent& event);
protected:
    wxDECLARE_EVENT_TABLE();
};

class BasicDrawPane
    : public wxPanel
{
 
public:
    BasicDrawPane(wxFrame* parent);
 
    void paintEvent(wxPaintEvent & evt);
    void paintNow();
 
    void render(wxDC& dc);

    void SetPoints(std::vector<Eigen::Vector2d> points);
    void SetLines(std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines);
private: 
    std::vector<Eigen::Vector2d> _points;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> _lines;

    //wxDECLARE_EVENT_TABLE();
};

class wxApplicationNode
    : public wxApp
{
public:
    virtual bool OnInit();
    void DoUpdate(wxIdleEvent &event);
 
private:
    void ExtractPointsAndLinesFromTreeForDrawPane();

    //wxFrame *frame;
    //BasicDrawPane *drawPane;
    wxGLCanvas * MyGLCanvas;
    PlannerWrapper *_planner;
 
    wxDECLARE_EVENT_TABLE();
};

wxGLCanvasSubClass::wxGLCanvasSubClass(wxFrame *parent)
:wxGLCanvas(parent, wxID_ANY,  wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas")){
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };
 
/*
NOTE: this example uses GLUT in order to have a free teapot model
to display, to show 3D capabilities. GLUT, however, seems to cause problems
on some systems. If you meet problems, first try commenting out glutInit(),
then try comeenting out all glut code
*/
    glutInit(&argc, argv);
}
 
 
void wxGLCanvasSubClass::Paintit(wxPaintEvent& WXUNUSED(event)){
    Render();
}
 
void wxGLCanvasSubClass::Render()
{
    SetCurrent();
    wxPaintDC(this);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);
 
    glBegin(GL_POLYGON);
        glColor3f(1.0, 1.0, 1.0);
        glVertex2f(-0.5, -0.5);
        glVertex2f(-0.5, 0.5);
        glVertex2f(0.5, 0.5);
        glVertex2f(0.5, -0.5);
        glColor3f(0.4, 0.5, 0.4);
        glVertex2f(0.0, -0.8);
    glEnd();
 
    glBegin(GL_POLYGON);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2f(0.1, 0.1);
        glVertex2f(-0.1, 0.1);
        glVertex2f(-0.1, -0.1);
        glVertex2f(0.1, -0.1);
    glEnd();
 
// using a little of glut
    glColor4f(0,0,1,1);
    glutWireTeapot(0.4);
 
    glLoadIdentity();
    glColor4f(2,0,1,1);
    glutWireTeapot(0.6);
// done using glut
 
    glFlush();
    SwapBuffers();
}

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
    const double r = 3;

    /// draw all points
    for(const Eigen::Vector2d& point : _points)
    {
        // circles
        dc.SetBrush(*wxGREEN_BRUSH); // green filling
        dc.SetPen( wxPen( wxColor(255,0,0), 2 ) ); // 2-pixels-thick red outline

        // draw a circle around p1 and p2
        dc.DrawCircle( wxPoint(point.x(),point.y()), r /* radius */ );
    }

    /// draw all lines
    for(const std::pair<Eigen::Vector2d, Eigen::Vector2d>& line : _lines)
    {
        // draw a line
        dc.SetPen( wxPen( wxColor(0,0,0), 1 ) ); // black line, 1 pixels thick
        dc.DrawLine( line.first.x(), line.first.y(), line.second.x(), line.second.y() );
    }
}

void BasicDrawPane::SetPoints(std::vector<Eigen::Vector2d> points)
{
    _points = points;
}

void BasicDrawPane::SetLines(std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines)
{
    _lines = lines;
}

////////////////////////////////////////////////////////////////////////////////////

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()

//wxBEGIN_EVENT_TABLE(BasicDrawPane, wxPanel)
//    EVT_PAINT(BasicDrawPane::paintEvent)
//wxEND_EVENT_TABLE()
 
wxBEGIN_EVENT_TABLE(wxGLCanvasSubClass, wxGLCanvas)
    EVT_PAINT(wxGLCanvasSubClass::Paintit)
wxEND_EVENT_TABLE()

IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

bool wxApplicationNode::OnInit()
{
/*
    _planner = new PlannerWrapper();

    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
    frame = new wxFrame((wxFrame *)NULL, -1,  wxT("BiRRT"), wxPoint(50,50), wxSize(800,600));
    
    drawPane = new BasicDrawPane( (wxFrame*) frame );
    sizer->Add(drawPane, 1, wxEXPAND);
 
    frame->SetSizer(sizer);
    frame->SetAutoLayout(true);
 
    frame->Show(true);
    return true;
*/
    wxFrame *frame = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(50,50), wxSize(200,200));
    new wxGLCanvasSubClass(frame);
 
    frame->Show(TRUE);
    return TRUE;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{
    if(ros::ok())
    {
        _planner->step();
        ros::spinOnce();
	ros::Duration(1).sleep();
    }

    //ExtractPointsAndLinesFromTreeForDrawPane();
    //drawPane->paintNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForDrawPane()
{
/*
    std::vector<Eigen::Vector2d> points;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines;

    /// startTree
    for(const RRT::Node<Eigen::Vector2d>& node : _planner->_biRRT->startTree().allNodes())
    {
	    Eigen::Vector2d current(node.state().x(), node.state().y());
        points.push_back(current);
        if(node.parent())
        {
	    Eigen::Vector2d parent(node.parent()->state().x(), node.parent()->state().y());
            std::pair<Eigen::Vector2d, Eigen::Vector2d> line(current, parent);

            points.push_back(parent);
            lines.push_back(line);
        }
    }

    /// goalTree
    for(const RRT::Node<Eigen::Vector2d>& node : _planner->_biRRT->goalTree().allNodes())
    {
	    Eigen::Vector2d current(node.state().x(), node.state().y());
        points.push_back(current);
        if(node.parent())
        {
	    Eigen::Vector2d parent(node.parent()->state().x(), node.parent()->state().y());
            std::pair<Eigen::Vector2d, Eigen::Vector2d> line(current, parent);

            points.push_back(parent);
            lines.push_back(line);
        }
    }

    //drawPane->SetPoints(points);
    //drawPane->SetLines(lines);*/
}

////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Starting RRT node with wxWidget visualization");

    wxEntryStart( argc, argv );
    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    /// running - loop see DoUpdate()

    wxTheApp->OnExit();
    wxEntryCleanup();

    return 0;
}
