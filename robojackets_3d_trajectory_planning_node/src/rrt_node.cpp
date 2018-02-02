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

// ----------------------------------------------------------------------------
// helper functions
// ----------------------------------------------------------------------------

static void CheckGLError()
{
    GLenum errLast = GL_NO_ERROR;

    for ( ;; )
    {
        GLenum err = glGetError();
        if ( err == GL_NO_ERROR )
            return;

        // normally the error is reset by the call to glGetError() but if
        // glGetError() itself returns an error, we risk looping forever here
        // so check that we get a different error than the last time
        if ( err == errLast )
        {
            wxLogError(wxT("OpenGL error state couldn't be reset."));
            return;
        }

        errLast = err;

        wxLogError(wxT("OpenGL error %d"), err);
    }
}

// function to draw the texture for cube faces
static wxImage DrawDice(int size, unsigned num)
{
    wxASSERT_MSG( num >= 1 && num <= 6, wxT("invalid dice index") );

    const int dot = size/16;        // radius of a single dot
    const int gap = 5*size/32;      // gap between dots

    wxBitmap bmp(size, size);
    wxMemoryDC dc;
    dc.SelectObject(bmp);
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();
    dc.SetBrush(*wxBLACK_BRUSH);

    // the upper left and lower right points
    if ( num != 1 )
    {
        dc.DrawCircle(gap + dot, gap + dot, dot);
        dc.DrawCircle(size - gap - dot, size - gap - dot, dot);
    }

    // draw the central point for odd dices
    if ( num % 2 )
    {
        dc.DrawCircle(size/2, size/2, dot);
    }

    // the upper right and lower left points
    if ( num > 3 )
    {
        dc.DrawCircle(size - gap - dot, gap + dot, dot);
        dc.DrawCircle(gap + dot, size - gap - dot, dot);
    }

    // finally those 2 are only for the last dice
    if ( num == 6 )
    {
        dc.DrawCircle(gap + dot, size/2, dot);
        dc.DrawCircle(size - gap - dot, size/2, dot);
    }

    dc.SelectObject(wxNullBitmap);

    return bmp.ConvertToImage();
}


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

// the rendering context used by all GL canvases
class TestGLContext : public wxGLContext
{
public:
    TestGLContext(wxGLCanvas *canvas);

    // render the cube showing it at given angles
    void DrawRotatedCube(float xangle, float yangle);

private:
    // textures for the cube faces
    GLuint m_textures[6];
};

class wxApplicationNode
    : public wxApp
{
public:
    wxApplicationNode() { m_glContext = NULL;}
    // Returns the shared context used by all frames and sets it as current for
    // the given canvas.
    TestGLContext& GetContext(wxGLCanvas *canvas);
    // virtual wxApp methods
    virtual bool OnInit();
    virtual int OnExit();
    
    void DoUpdate(wxIdleEvent &event);
 
private:
    void ExtractPointsAndLinesFromTreeForCanvas();

    //wxFrame *frame;
    PlannerWrapper *_planner;
    // the GL context we use for all our mono rendering windows
    TestGLContext *m_glContext;
 
    wxDECLARE_EVENT_TABLE();
};


////////////////////////////////////////////////////////////////////////////////////

typedef Eigen::Vector2d           Point;
typedef std::pair<Point, Point>   Line;
typedef std::pair<Point, wxColor> PointWithColor;
typedef std::pair<Line, wxColor>  LineWithColor;

class TreeToDraw
{
public:
    TreeToDraw() {};
    std::vector<PointWithColor> colored_points_;
    std::vector<LineWithColor> colored_lines_;
};

// Define a new frame type
class MyFrame : public wxFrame
{
public:
    MyFrame();
    
private:    
    void OnClose(wxCommandEvent& event);

    wxDECLARE_EVENT_TABLE();
};

class TestGLCanvas : public wxGLCanvas
{
public:
    TestGLCanvas(wxWindow *parent, int *attribList = NULL);
    
    void ClearTrees();
    void AddTree(TreeToDraw tree);

private:
    std::vector<Eigen::Vector2d> points_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines_;    
    std::vector<TreeToDraw> trees_;

    // angles of rotation around x- and y- axis
    float m_xangle,
          m_yangle;

    void OnPaint(wxPaintEvent& event);

    wxDECLARE_EVENT_TABLE();
};

void TestGLCanvas::ClearTrees()
{
    trees_.clear();
}

void TestGLCanvas::AddTree(TreeToDraw tree)
{
    trees_.push_back(tree);
}

////////////////////////////////////////////////////////////////////////////////////

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()
    
IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

bool wxApplicationNode::OnInit()
{
    ROS_INFO("START wxApplicationNode::OnInit()");

    _planner = new PlannerWrapper();
    
    if ( !wxApp::OnInit() )
        return false;

    new MyFrame();
    
    ROS_INFO("END wxApplicationNode::OnInit()");

    return true;
}

int wxApplicationNode::OnExit()
{
    delete m_glContext;
    
    ROS_INFO("END wxApplicationNode::OnExit()");

    return wxApp::OnExit();
}

TestGLContext& wxApplicationNode::GetContext(wxGLCanvas *canvas)
{
    ROS_INFO("START wxApplicationNode::GetContext()");
    
    TestGLContext *glContext;
    
    if ( !m_glContext )
    {
        // Create the OpenGL context for the first mono window which needs it:
        // subsequently created windows will all share the same context.
        m_glContext = new TestGLContext(canvas);
    }
    glContext = m_glContext;

    glContext->SetCurrent(*canvas);
    
    ROS_INFO("END wxApplicationNode::GetContext()");

    return *glContext;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{
    ROS_INFO("START wxApplicationNode::DoUpdate()");
    
    if(ros::ok())
    {
        _planner->step();
        ros::spinOnce();
	    ros::Duration(1).sleep();
    }

    ExtractPointsAndLinesFromTreeForCanvas();
    //drawPane->paintNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
    
    ROS_INFO("END wxApplicationNode::DoUpdate()");
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForCanvas()
{
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
    //drawPane->SetLines(lines);
}

// ----------------------------------------------------------------------------
// TestGLCanvas
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(TestGLCanvas, wxGLCanvas)
    EVT_PAINT(TestGLCanvas::OnPaint)
wxEND_EVENT_TABLE()

TestGLCanvas::TestGLCanvas(wxWindow *parent, int *attribList)
    // With perspective OpenGL graphics, the wxFULL_REPAINT_ON_RESIZE style
    // flag should always be set, because even making the canvas smaller should
    // be followed by a paint event that updates the entire canvas with new
    // viewport settings.
    : wxGLCanvas(parent, wxID_ANY, attribList,
                 wxDefaultPosition, wxDefaultSize,
                 wxFULL_REPAINT_ON_RESIZE),
      m_xangle(30.0),
      m_yangle(30.0)
{
}

void TestGLCanvas::OnPaint(wxPaintEvent& WXUNUSED(event))
{    
    // This is required even though dc is not used otherwise.
    wxPaintDC dc(this);

    // Set the OpenGL viewport according to the client size of this canvas.
    // This is done here rather than in a wxSizeEvent handler because our
    // OpenGL rendering context (and thus viewport setting) is used with
    // multiple canvases: If we updated the viewport in the wxSizeEvent
    // handler, changing the size of one canvas causes a viewport setting that
    // is wrong when next another canvas is repainted.
    const wxSize ClientSize = GetClientSize();

    TestGLContext& canvas = wxGetApp().GetContext(this);
    glViewport(0, 0, ClientSize.x, ClientSize.y);

    // Render the graphics and swap the buffers.
    GLboolean quadStereoSupported;
    glGetBooleanv( GL_STEREO, &quadStereoSupported);
    if ( quadStereoSupported )
    {
        glDrawBuffer( GL_BACK_LEFT );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.47f, 0.53f, -0.5f, 0.5f, 1.0f, 3.0f);
        canvas.DrawRotatedCube(m_xangle, m_yangle);
        CheckGLError();
        glDrawBuffer( GL_BACK_RIGHT );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.53f, 0.47f, -0.5f, 0.5f, 1.0f, 3.0f);
        canvas.DrawRotatedCube(m_xangle, m_yangle);
        CheckGLError();
    }
    else
    {
        canvas.DrawRotatedCube(m_xangle, m_yangle);
    }
    SwapBuffers();
}

wxString glGetwxString(GLenum name)
{    
    const GLubyte *v = glGetString(name);
    if ( v == 0 )
    {
        // The error is not important. It is GL_INVALID_ENUM.
        // We just want to clear the error stack.
        glGetError();

        return wxString();
    }

    return wxString((const char*)v);
}

// ----------------------------------------------------------------------------
// MyFrame: main application window
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(wxID_CLOSE, MyFrame::OnClose)
wxEND_EVENT_TABLE()

MyFrame::MyFrame()
    : wxFrame(NULL, wxID_ANY, wxT("wxWidgets OpenGL"))
{
    new TestGLCanvas(this, NULL);


    SetClientSize(400, 400);
    Show();

    // test IsDisplaySupported() function:
    static const int attribs[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, 0 };
    wxLogStatus("Double-buffered display %s supported",
                wxGLCanvas::IsDisplaySupported(attribs) ? "is" : "not");
}

void MyFrame::OnClose(wxCommandEvent& WXUNUSED(event))
{
    // true is to force the frame to close
    Close(true);
}

// ============================================================================
// implementation
// ============================================================================

// ----------------------------------------------------------------------------
// TestGLContext
// ----------------------------------------------------------------------------

TestGLContext::TestGLContext(wxGLCanvas *canvas)
             : wxGLContext(canvas)
{    
    SetCurrent(*canvas);

    // set up the parameters we want to use
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_TEXTURE_2D);

    // add slightly more light, the default lighting is rather dark
    GLfloat ambient[] = { 0.5, 0.5, 0.5, 0.5 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);

    // set viewing projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-0.5f, 0.5f, -0.5f, 0.5f, 1.0f, 3.0f);

    // create the textures to use for cube sides: they will be reused by all
    // canvases (which is probably not critical in the case of simple textures
    // we use here but could be really important for a real application where
    // each texture could take many megabytes)
    glGenTextures(WXSIZEOF(m_textures), m_textures);

    for ( unsigned i = 0; i < WXSIZEOF(m_textures); i++ )
    {
        glBindTexture(GL_TEXTURE_2D, m_textures[i]);

        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        const wxImage img(DrawDice(256, i + 1));

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.GetWidth(), img.GetHeight(),
                     0, GL_RGB, GL_UNSIGNED_BYTE, img.GetData());
    }

    CheckGLError();
}

void TestGLContext::DrawRotatedCube(float xangle, float yangle)
{    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -2.0f);
    glRotatef(xangle, 1.0f, 0.0f, 0.0f);
    glRotatef(yangle, 0.0f, 1.0f, 0.0f);

    // draw six faces of a cube of size 1 centered at (0, 0, 0)
    glBindTexture(GL_TEXTURE_2D, m_textures[0]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 0.0f, 1.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f,-0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f,-0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, m_textures[1]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 0.0f,-1.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f, 0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f, 0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f,-0.5f,-0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, m_textures[2]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 1.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f, 0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f, 0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f, 0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, m_textures[3]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f,-1.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f,-0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f,-0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, m_textures[4]);
    glBegin(GL_QUADS);
        glNormal3f( 1.0f, 0.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f,-0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f,-0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f, 0.5f,-0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, m_textures[5]);
    glBegin(GL_QUADS);
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f,-0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f, 0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f, 0.5f,-0.5f);
    glEnd();
    
    
    /*
    //  node drawing radius
    const double r = 3;
    for(const TreeToDraw& tree : trees_)
    {
        /// draw all points
        for(const PointWithColor& colored_point : tree.colored_points_)
        {
            // circles
            dc.SetPen( wxPen( colored_point.second, 2 ) ); // 2-pixels thick outline

            // draw a circle around
            dc.DrawCircle( wxPoint(colored_point.first.x(),colored_point.first.y()), r );
        }

        /// draw all lines
        for(const LineWithColor& colored_line : tree.colored_lines_)
        {
            // draw a line
            dc.SetPen( wxPen( colored_line.second, 1 ) ); // 1 pixels thick
            dc.DrawLine( colored_line.first.first.x(), colored_line.first.first.y(), colored_line.first.second.x(), colored_line.first.second.y() );
        }
    }*/

    glFlush();

    CheckGLError();
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
