#include "wxApplication.h"
#include <string>
#include <ros/ros.h>

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

    ExtractPointsAndLinesFromTreeForContext();
    GetFrame()->GetCanvas()->DrawNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForContext()
{
    if(NULL != context_)
    {
        context_->ClearTrees();

        /// startTree
        TreeToDraw* start_tree = new TreeToDraw();
        wxColor start_tree_color(255,0,0);
        for(const RRT::Node<Eigen::Vector3d>& node : planner_->birrt_->startTree().allNodes())
        {
            Point current_node_point(node.state().x(),
                                     node.state().y(),
                                     node.state().z());
            PointWithColor current_colored_node_point(current_node_point, start_tree_color);
            start_tree->colored_points_.push_back(current_colored_node_point);

            if(node.parent())
            {
                Point parent_node_point(node.parent()->state().x(),
                                        node.parent()->state().y(),
                                        node.parent()->state().z());
                PointWithColor parent_colored_node_point(parent_node_point, start_tree_color);
                Line line(current_node_point, parent_node_point);
                LineWithColor colored_line(line, start_tree_color);

                start_tree->colored_points_.push_back(parent_colored_node_point);
                start_tree->colored_lines_.push_back(colored_line);
            }
        }

        context_->AddTree(*start_tree);
        delete start_tree;

        /// goalTree
        TreeToDraw* goal_tree = new TreeToDraw();
        wxColor goal_tree_color(0,0,255);
        for(const RRT::Node<Eigen::Vector3d>& node : planner_->birrt_->goalTree().allNodes())
        {
            Point current_node_point(node.state().x(),
                                     node.state().y(),
                                     node.state().z());
            PointWithColor current_colored_node_point(current_node_point, goal_tree_color);
            goal_tree->colored_points_.push_back(current_colored_node_point);

            if(node.parent())
            {
                Point parent_node_point(node.parent()->state().x(),
                                        node.parent()->state().y(),
                                        node.parent()->state().z());
                PointWithColor parent_colored_node_point(parent_node_point, goal_tree_color);
                Line line(current_node_point, parent_node_point);
                LineWithColor colored_line(line, goal_tree_color);

                goal_tree->colored_points_.push_back(parent_colored_node_point);
                goal_tree->colored_lines_.push_back(colored_line);
            }
        }

        context_->AddTree(*goal_tree);
        delete goal_tree;
    }
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
      view_azimuth_(0),
      view_elevation_(0),
      radius_(0.5*(khalf_width+khalf_height)),
      view_look_at_point_x_(khalf_width),
      view_look_at_point_y_(khalf_height),
      view_look_at_point_z_(0)
{
}

double RRTGLCanvas::GetAzimuth()
{
    return view_azimuth_;
}

double RRTGLCanvas::GetElevation()
{
    return view_elevation_;
}

double RRTGLCanvas::GetRadius()
{
    return radius_;
}

double RRTGLCanvas::GetLookAtPointX()
{
    return view_look_at_point_x_;
}

double RRTGLCanvas::GetLookAtPointY()
{
    return view_look_at_point_y_;
}

double RRTGLCanvas::GetLookAtPointZ()
{
    return view_look_at_point_z_;
}

void RRTGLCanvas::SetAzimuth(double azimuth)
{
    view_azimuth_ = azimuth;
}

void RRTGLCanvas::SetElevation(double elevation)
{
    view_elevation_ = elevation;
}

void RRTGLCanvas::SetRadius(double radius)
{
    radius_ = radius;
}

void RRTGLCanvas::SetLookAtPointX(double look_at_x)
{
    view_look_at_point_x_ = look_at_x;
}

void RRTGLCanvas::SetLookAtPointY(double look_at_y)
{
    view_look_at_point_y_ = look_at_y;
}

void RRTGLCanvas::SetLookAtPointZ(double look_at_z)
{
    view_look_at_point_z_ = look_at_z;
}

void RRTGLCanvas::ResetView()
{
    SetAzimuth(0);
    SetElevation(0);
    SetRadius(0.5*(khalf_width+khalf_height));
    SetLookAtPointX(khalf_width);
    SetLookAtPointY(khalf_height);
    SetLookAtPointZ(0);

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::Spin(double azimuth, double elevation)
{
    SetAzimuth(azimuth + GetAzimuth());
    SetElevation(elevation + GetElevation());

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::Translate(double translate_x, double translate_y, double translate_z)
{
    SetLookAtPointX(translate_x + GetLookAtPointX());
    SetLookAtPointY(translate_y + GetLookAtPointY());
    SetLookAtPointZ(translate_z + GetLookAtPointZ());

    DrawNow();
    Refresh(false);
}

void RRTGLCanvas::ChangeRadius(double delta_radius)
{
    SetRadius(delta_radius + GetRadius());

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
    context_canvas.SetAzimuth(GetAzimuth());
    context_canvas.SetElevation(GetElevation());
    context_canvas.SetRadius(GetRadius());
    context_canvas.SetLookAtPointX(GetLookAtPointX());
    context_canvas.SetLookAtPointY(GetLookAtPointY());
    context_canvas.SetLookAtPointZ(GetLookAtPointZ());
    context_canvas.DrawNow();

    // Then draw
    SwapBuffers();
}

void RRTGLCanvas::OnKeyDown(wxKeyEvent& event)
{
    double delta_angle = kdelta_angle;
    double delta_translation = kdelta_translation;
    switch ( event.GetKeyCode() )
    {
        case WXK_LEFT: // move look-at-point along x
            Translate(+delta_translation, 0.0f, 0.0f);
            break;

        case WXK_RIGHT: // move look-at-point along x
            Translate(-delta_translation, 0.0f, 0.0f);
            break;

        case WXK_UP: // move look-at-point along y
            Translate(0.0f, +delta_translation, 0.0f);
            break;

        case WXK_DOWN: // move look-at-point along y
            Translate(0.0f, -delta_translation, 0.0f);
            break;

        case WXK_F1: // decrease azimuth
            Spin(-delta_angle, 0.0f);
            break;

        case WXK_F2: // increase azimuth
            Spin(+delta_angle, 0.0f);
            break;

        case WXK_F3: // decrease elevation
            Spin(0.0f, -delta_angle);
            break;

        case WXK_F4: // increase elevation
            Spin(0.0f, +delta_angle);
            break;

        case WXK_F5: // increase radius
            ChangeRadius(+delta_translation);
            break;

        case WXK_F6: // decrease radius
            ChangeRadius(-delta_translation);
            break;

        case WXK_SPACE: // reset
            ResetView();
            break;

        default:
            return;
    }
}

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

// ----------------------------------------------------------------------------
// RRTGLContext
// ----------------------------------------------------------------------------

RRTGLContext::RRTGLContext(wxGLCanvas *canvas)
    : wxGLContext(canvas),
      view_azimuth_(0),
      view_elevation_(0),
      radius_(0.5*(khalf_width+khalf_height)),
      view_look_at_point_x_(khalf_width),
      view_look_at_point_y_(khalf_height),
      view_look_at_point_z_(0)
{
    SetCurrent(*canvas);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90,(double)khalf_width/khalf_height,1,5*(khalf_width+khalf_height));

    CheckGLError();
}

void RRTGLContext::DrawNow()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    double radius = GetRadius(); // Straight line distance between the camera and look at point

    // Calculate the camera position using the radius and angles
    double camera_position_x = radius * -sin(GetAzimuth()*(M_PI/180)) * cos((GetElevation())*(M_PI/180));
    double camera_position_y = radius * -sin((GetElevation())*(M_PI/180));
    double camera_position_z = -radius * cos((GetAzimuth())*(M_PI/180)) * cos((GetElevation())*(M_PI/180));

    camera_position_x += GetLookAtPointX();
    camera_position_y += GetLookAtPointY();
    camera_position_z += GetLookAtPointZ();

    // Set the camera position and lookat point
    gluLookAt(camera_position_x, camera_position_y,camera_position_z,   // Camera position
              GetLookAtPointX(), GetLookAtPointY(), GetLookAtPointZ(),    // Look at point
              0.0, 1.0, 0.0);   // Up vector

    glPointSize(5.0f);//set point size to 5 pixels

    for(const TreeToDraw& tree : trees_)
    {
        glColor3f(0.0f,0.0f,1.0f); //blue color as default

        glBegin(GL_POINTS); //starts drawing of points
            /// draw all points
            for(const PointWithColor& colored_point : tree.colored_points_)
            {
                glColor3f(colored_point.second.Red()/255,
                          colored_point.second.Green()/255,
                          colored_point.second.Blue()/255);
                glVertex3f(colored_point.first.x(),
                           colored_point.first.y(),
                           colored_point.first.z());
            }
        glEnd(); //end drawing of points

        glColor3f(0.0f,0.0f,1.0f); //blue color as default

        glBegin(GL_LINES); //starts drawing of lines
            /// draw all lines
            for(const LineWithColor& colored_line : tree.colored_lines_)
            {
                glColor3f(colored_line.second.Red()/255,
                          colored_line.second.Green()/255,
                          colored_line.second.Blue()/255);
                glVertex3f(colored_line.first.first.x(),
                           colored_line.first.first.y(),
                           colored_line.first.first.z());
                glVertex3f(colored_line.first.second.x(),
                           colored_line.first.second.y(),
                           colored_line.first.second.z());
            }
        glEnd(); //end drawing of lines
    }

    glDisable(GL_POINT_SMOOTH);
    glBlendFunc(GL_NONE, GL_NONE);
    glDisable(GL_BLEND);

    glFlush();

    glPopAttrib();

    CheckGLError();
}

void RRTGLContext::ClearTrees()
{
    trees_.clear();
}

void RRTGLContext::AddTree(TreeToDraw tree)
{
    trees_.push_back(tree);
}

double RRTGLContext::GetAzimuth()
{
    return view_azimuth_;
}

double RRTGLContext::GetElevation()
{
    return view_elevation_;
}

double RRTGLContext::GetRadius()
{
    return radius_;
}

double RRTGLContext::GetLookAtPointX()
{
    return view_look_at_point_x_;
}

double RRTGLContext::GetLookAtPointY()
{
    return view_look_at_point_y_;
}

double RRTGLContext::GetLookAtPointZ()
{
    return view_look_at_point_z_;
}

void RRTGLContext::SetAzimuth(double azimuth)
{
    view_azimuth_ = azimuth;
}

void RRTGLContext::SetElevation(double elevation)
{
    view_elevation_ = elevation;
}

void RRTGLContext::SetRadius(double radius)
{
    radius_ = radius;
}

void RRTGLContext::SetLookAtPointX(double look_at_x)
{
    view_look_at_point_x_ = look_at_x;
}

void RRTGLContext::SetLookAtPointY(double look_at_y)
{
    view_look_at_point_y_ = look_at_y;
}

void RRTGLContext::SetLookAtPointZ(double look_at_z)
{
    view_look_at_point_z_ = look_at_z;
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
