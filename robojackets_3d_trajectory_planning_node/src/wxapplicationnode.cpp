#include "robojackets_3d_trajectory_planning_node/wxapplicationnode.h"
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

PlannerWrapper* wxApplicationNode::GetPlanner()
{
    return planner_;
}

RRTGLContext& wxApplicationNode::GetContext(wxGLCanvas *canvas)
{
    
    RRTGLContext *rrt_gl_context;
    
    if ( !context_ )
    {
        // Create the OpenGL context for the first mono window which needs it:
        // subsequently created windows will all share the same context.
        context_ = new RRTGLContext(canvas);
    }
    rrt_gl_context = context_;

    rrt_gl_context->SetCurrent(*canvas);

    return *rrt_gl_context;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{
    if(ros::ok())
    {
        planner_->Step();
        ros::spinOnce();
	    ros::Duration(1).sleep();
    }

    ExtractStartAndGoalStateForContext();
    ExtractPointsAndLinesFromTreeForContext();
    GetFrame()->GetCanvas()->DrawNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}

void wxApplicationNode::ExtractStartAndGoalStateForContext()
{
    if(NULL != context_)
    {
        context_->SetStartPoint(planner_->birrt_->GetStartState());
        context_->SetGoalPoint(planner_->birrt_->GetGoalState());
    }
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForContext()
{
    if(NULL != context_)
    {
        context_->ClearTrees();

        /// StartTree
        TreeToDraw* start_tree = new TreeToDraw();
        wxColor start_tree_color(255,0,0);
        for(const RRT::Node<Eigen::Vector3d>& node : planner_->birrt_->GetStartTree().GetAllNodes())
        {
            Point current_node_point(node.GetState().x(),
                                     node.GetState().y(),
                                     node.GetState().z());
            PointWithColor current_colored_node_point(current_node_point, start_tree_color);
            start_tree->colored_points_.push_back(current_colored_node_point);

            if(node.GetParent())
            {
                Point parent_node_point(node.GetParent()->GetState().x(),
                                        node.GetParent()->GetState().y(),
                                        node.GetParent()->GetState().z());
                PointWithColor parent_colored_node_point(parent_node_point, start_tree_color);
                Line line(current_node_point, parent_node_point);
                LineWithColor colored_line(line, start_tree_color);

                start_tree->colored_points_.push_back(parent_colored_node_point);
                start_tree->colored_lines_.push_back(colored_line);
            }
        }

        context_->AddTree(*start_tree);
        delete start_tree;

        /// GoalTree
        TreeToDraw* goal_tree = new TreeToDraw();
        wxColor goal_tree_color(0,0,255);
        for(const RRT::Node<Eigen::Vector3d>& node : planner_->birrt_->GetGoalTree().GetAllNodes())
        {
            Point current_node_point(node.GetState().x(),
                                     node.GetState().y(),
                                     node.GetState().z());
            PointWithColor current_colored_node_point(current_node_point, goal_tree_color);
            goal_tree->colored_points_.push_back(current_colored_node_point);

            if(node.GetParent())
            {
                Point parent_node_point(node.GetParent()->GetState().x(),
                                        node.GetParent()->GetState().y(),
                                        node.GetParent()->GetState().z());
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
      view_look_at_point_z_(khalf_depth)
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
    SetLookAtPointZ(khalf_depth);

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
