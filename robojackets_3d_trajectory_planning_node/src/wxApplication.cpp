#include "wxApplication.hpp"

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()
wxBEGIN_EVENT_TABLE(RRTGLCanvas, wxGLCanvas)
    EVT_PAINT(RRTGLCanvas::OnPaint)
    EVT_KEY_DOWN(RRTGLCanvas::OnKeyDown)
wxEND_EVENT_TABLE()
wxBEGIN_EVENT_TABLE(RRTFrame, wxFrame)
    EVT_MENU(wxID_CLOSE, RRTFrame::OnClose)
wxEND_EVENT_TABLE()
    
IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

////////////////////////////////////////////////////////////////////////////////////


// ----------------------------------------------------------------------------
// wxApplicationNode
// ----------------------------------------------------------------------------

bool wxApplicationNode::OnInit()
{
    planner_ = new PlannerWrapper();
    
    if ( !wxApp::OnInit() )
        return false;

    frame_ = new RRTFrame();

    return true;
}

int wxApplicationNode::OnExit()
{
    delete context_;

    return wxApp::OnExit();
}

RRTFrame* wxApplicationNode::GetFrame()
{
    return frame_;
}

RRTGLContext& wxApplicationNode::GetContext(wxGLCanvas *canvas)
{
    
    RRTGLContext *glContext;
    
    if ( !context_ )
    {
        // Create the OpenGL context for the first mono window which needs it:
        // subsequently created windows will all share the same context.
        context_ = new RRTGLContext(canvas);
    }
    glContext = context_;

    glContext->SetCurrent(*canvas);

    return *glContext;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{    
    if(ros::ok())
    {
        planner_->step();
        ros::spinOnce();
	    ros::Duration(1).sleep();
    }

    ExtractPointsAndLinesFromTreeForCanvas();
    GetFrame()->GetCanvas()->DrawNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForCanvas()
{
    GetFrame()->GetCanvas()->ClearTrees();

    /// startTree
    TreeToDraw* start_tree = new TreeToDraw();
    wxColor start_tree_color(255,0,0);
    for(const RRT::Node<Eigen::Vector2d>& node : planner_->birrt_->startTree().allNodes())
    {
	    Point current_node_point(node.state().x(), node.state().y());
        PointWithColor current_colored_node_point(current_node_point, start_tree_color);
        start_tree->colored_points_.push_back(current_colored_node_point);

        if(node.parent())
        {
	        Point parent_node_point(node.parent()->state().x(), node.parent()->state().y());
            PointWithColor parent_colored_node_point(parent_node_point, start_tree_color);
            Line line(current_node_point, parent_node_point);
            LineWithColor colored_line(line, start_tree_color);
            
            start_tree->colored_points_.push_back(parent_colored_node_point);
            start_tree->colored_lines_.push_back(colored_line);
        }
    }

    GetFrame()->GetCanvas()->AddTree(*start_tree);
    delete start_tree;
    
    /// goalTree
    TreeToDraw* goal_tree = new TreeToDraw();
    wxColor goal_tree_color(0,0,255);
    for(const RRT::Node<Eigen::Vector2d>& node : planner_->birrt_->goalTree().allNodes())
    {
	    Point current_node_point(node.state().x(), node.state().y());
        PointWithColor current_colored_node_point(current_node_point, goal_tree_color);
        goal_tree->colored_points_.push_back(current_colored_node_point);

        if(node.parent())
        {
	        Point parent_node_point(node.parent()->state().x(), node.parent()->state().y());
            PointWithColor parent_colored_node_point(parent_node_point, goal_tree_color);
            Line line(current_node_point, parent_node_point);
            LineWithColor colored_line(line, goal_tree_color);
            
            goal_tree->colored_points_.push_back(parent_colored_node_point);
            goal_tree->colored_lines_.push_back(colored_line);
        }
    }

    GetFrame()->GetCanvas()->AddTree(*goal_tree);
    delete goal_tree;
}

// ----------------------------------------------------------------------------
// RRTGLCanvas
// ----------------------------------------------------------------------------

RRTGLCanvas::RRTGLCanvas(wxWindow *parent, int *attribList)
    // With perspective OpenGL graphics, the wxFULL_REPAINT_ON_RESIZE style
    // flag should always be set, because even making the canvas smaller should
    // be followed by a paint event that updates the entire canvas with new
    // viewport settings.
    : wxGLCanvas(parent, wxID_ANY, attribList,
                 wxDefaultPosition, wxDefaultSize,
                 wxFULL_REPAINT_ON_RESIZE),
      view_rotation_angle_x_(0),
      view_rotation_angle_y_(0),
      view_rotation_angle_z_(0),
      view_translation_x_(0),
      view_translation_y_(0),
      view_translation_z_(-2)
{
}

double RRTGLCanvas::GetAngleX()
{
    return view_rotation_angle_x_;
}

double RRTGLCanvas::GetAngleY()
{
    return view_rotation_angle_y_;
}

double RRTGLCanvas::GetAngleZ()
{
    return view_rotation_angle_z_;
}

double RRTGLCanvas::GetTranslationX()
{
    return view_translation_x_;
}

double RRTGLCanvas::GetTranslationY()
{
    return view_translation_y_;
}

double RRTGLCanvas::GetTranslationZ()
{
    return view_translation_z_;
}

void RRTGLCanvas::SetAngleX(double angle_x)
{
    view_rotation_angle_x_ = angle_x;
}

void RRTGLCanvas::SetAngleY(double angle_y)
{
    view_rotation_angle_y_ = angle_y;
}

void RRTGLCanvas::SetAngleZ(double angle_z)
{
    view_rotation_angle_z_ = angle_z;
}

void RRTGLCanvas::SetTranslationX(double translation_x)
{
    view_translation_x_ = translation_x;
}

void RRTGLCanvas::SetTranslationY(double translation_y)
{
    view_translation_y_ = translation_y;
}

void RRTGLCanvas::SetTranslationZ(double translation_z)
{
    view_translation_z_ = translation_z;
}

void RRTGLCanvas::ResetView()
{
    SetAngleX(0);
    SetAngleY(0);
    SetAngleZ(0);
    SetTranslationX(0);
    SetTranslationY(0);
    SetTranslationZ(0);

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::Spin(double angle_x, double angle_y, double angle_z)
{
    SetAngleX(angle_x + GetAngleX());
    SetAngleY(angle_y + GetAngleY());
    SetAngleZ(angle_z + GetAngleZ());

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::Translate(double translate_x, double translate_y, double translate_z)
{
    SetTranslationX(translate_x + GetTranslationX());
    SetTranslationY(translate_y + GetTranslationY());
    SetTranslationZ(translate_z + GetTranslationZ());

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::OnPaint(wxPaintEvent& WXUNUSED(event))
{    
    DrawNow();
}

void RRTGLCanvas::DrawNow()
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

    RRTGLContext& context_canvas = wxGetApp().GetContext(this);
    glViewport(0, 0, ClientSize.x, ClientSize.y);

    // Update angles and translation before drawing
    context_canvas.SetAngleX(GetAngleX());
    context_canvas.SetAngleY(GetAngleY());
    context_canvas.SetAngleZ(GetAngleZ());
    context_canvas.SetTranslationX(GetTranslationX());
    context_canvas.SetTranslationY(GetTranslationY());
    context_canvas.SetTranslationZ(GetTranslationZ());
    context_canvas.DrawNow();

    // Then draw
    SwapBuffers();
}

void RRTGLCanvas::OnKeyDown(wxKeyEvent& event)
{
    double delta_angle = 5.0;
    double delta_translation = 1.0;
    switch ( event.GetKeyCode() )
    {
        case WXK_LEFT: // move left
            Translate(0.0f, +delta_translation, 0.0f);
            break;

        case WXK_RIGHT: // move right
            Translate(0.0f, -delta_translation, 0.0f);
            break;

        case WXK_UP: // move forward
            Translate(0.0f, 0.0f, +delta_translation);
            break;

        case WXK_DOWN: // move back
            Translate(0.0f, 0.0f, -delta_translation);
            break;

        case WXK_NUMPAD4: // roll counter-clockwise
            Spin(0.0f, 0.0f, -delta_angle);
            break;

        case WXK_NUMPAD6: // roll clockwise
            Spin(0.0f, 0.0f, +delta_angle);
            break;

        case WXK_NUMPAD8: // tilt downwards
            Spin(0.0f, -delta_angle, 0.0f);
            break;

        case WXK_NUMPAD2: // tilt upwards
            Spin(0.0f, +delta_angle, 0.0f);
            break;

        case WXK_SPACE: // reset
            ResetView();
            break;

        default:
            return;
    }
}

void RRTGLCanvas::ClearTrees()
{
    trees_.clear();
}

void RRTGLCanvas::AddTree(TreeToDraw tree)
{
    trees_.push_back(tree);
}

// ----------------------------------------------------------------------------
// RRTFrame: main application window
// ----------------------------------------------------------------------------

RRTFrame::RRTFrame()
    : wxFrame(NULL, wxID_ANY, wxT("wxWidgets OpenGL"))
{
    canvas_ = new RRTGLCanvas(this, NULL);


    SetClientSize(800, 800);
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

// ----------------------------------------------------------------------------
// RRTGLContext
// ----------------------------------------------------------------------------

RRTGLContext::RRTGLContext(wxGLCanvas *canvas)
    : wxGLContext(canvas),
      view_rotation_angle_x_(0),
      view_rotation_angle_y_(0),
      view_rotation_angle_z_(0),
      view_translation_x_(0),
      view_translation_y_(0),
      view_translation_z_(0)
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

    CheckGLError();
}

void RRTGLContext::DrawNow()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(GetTranslationX(), GetTranslationY(), GetTranslationZ());
    glRotatef(GetAngleX(), 1.0f, 0.0f, 0.0f);
    glRotatef(GetAngleY(), 0.0f, 1.0f, 0.0f);
    glRotatef(GetAngleZ(), 0.0f, 0.0f, 1.0f);
    
    
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
    
    glColor3f(0.0f,0.0f,1.0f); //blue color
    
    glPointSize(20.0f);//set point size to 20 pixels
    
    glBegin(GL_POINTS); //starts drawing of points
        glVertex3f(1.0f,1.0f,0.0f);//upper-right corner
        glVertex3f(-1.0f,-1.0f,0.0f);//lower-left corner
    glEnd();//end drawing of points
    
    glBegin(GL_LINES); //starts drawing of lines
        glVertex3f(1.0f,1.0f,0.0f);//upper-right corner
        glVertex3f(-1.0f,-1.0f,0.0f);//lower-left corner
    glEnd();//end drawing of lines

    glFlush();

    CheckGLError();
}

double RRTGLContext::GetAngleX()
{
    return view_rotation_angle_x_;
}

double RRTGLContext::GetAngleY()
{
    return view_rotation_angle_y_;
}

double RRTGLContext::GetAngleZ()
{
    return view_rotation_angle_z_;
}

double RRTGLContext::GetTranslationX()
{
    return view_translation_x_;
}

double RRTGLContext::GetTranslationY()
{
    return view_translation_y_;
}

double RRTGLContext::GetTranslationZ()
{
    return view_translation_z_;
}

void RRTGLContext::SetAngleX(double angle_x)
{
    view_rotation_angle_x_ = angle_x;
}

void RRTGLContext::SetAngleY(double angle_y)
{
    view_rotation_angle_y_ = angle_y;
}

void RRTGLContext::SetAngleZ(double angle_z)
{
    view_rotation_angle_z_ = angle_z;
}

void RRTGLContext::SetTranslationX(double translation_x)
{
    view_translation_x_ = translation_x;
}

void RRTGLContext::SetTranslationY(double translation_y)
{
    view_translation_y_ = translation_y;
}

void RRTGLContext::SetTranslationZ(double translation_z)
{
    view_translation_z_ = translation_z;
}

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
