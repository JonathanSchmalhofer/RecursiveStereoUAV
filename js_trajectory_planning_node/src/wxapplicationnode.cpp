#include "js_trajectory_planning_node/wxapplicationnode.h"
#include <string>
#include <ros/ros.h>

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::UpdateDrawnCanvas)
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

extern bool external_trigger_subscriber_pcl;            // input
extern bool external_trigger_subscriber_pose;           // input
extern bool external_trigger_publisher;                 // output

extern sensor_msgs::PointCloud current_point_cloud;     // input
extern geometry_msgs::Pose current_pose;                // input
extern js_messages::Trajectory3D current_trajectory_3d; // output

extern double goal_x;
extern double goal_y;
extern double goal_z;

extern double size_x;
extern double size_y;
extern double size_z;

void ExternalTriggerPublisher();



bool wxApplicationNode::OnInit()
{
    planner_ = new PlannerWrapper(Eigen::Vector3d(size_x,size_y,size_z),  // size
                                  Eigen::Vector3d(current_pose.position.x,current_pose.position.y,current_pose.position.z),  // start
                                  Eigen::Vector3d(goal_x,goal_y,goal_z)); // goal

    if ( !wxApp::OnInit() )
        return false;

    frame_ = new RRTFrame();
    
    return true;
}

int wxApplicationNode::OnRun()
{
    return wxApp::OnRun();
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

void wxApplicationNode::GetTrajectory3D()
{
    // Clear array
    current_trajectory_3d.trajectory.clear();
    
    // Header
    current_trajectory_3d.header.stamp = ros::Time::now();
    current_trajectory_3d.header.frame_id = "";
    
    // Points
    for (int i = 0; i < 10; i++)
    {
        js_messages::Trajectory3DPointStamped new_point;
        // Point
        new_point.pose.position.x = i;
        new_point.pose.position.y = i;
        new_point.pose.position.z = i;
        // Orientation
        new_point.pose.orientation.x = 0;
        new_point.pose.orientation.y = 0;
        new_point.pose.orientation.z = 0;
        new_point.pose.orientation.w = 1;
        
        // push point to the trajectory
        current_trajectory_3d.trajectory.push_back(new_point);
    }
}

void wxApplicationNode::PointCloudCallback()
{
    planner_->Reset();
    PoseCallback();
    InjectCurrentPointCloudAsObstacles();
    planner_->Step();
    GetTrajectory3D();
    external_trigger_publisher = true;
    ExternalTriggerPublisher();
}

void wxApplicationNode::PoseCallback()
{
    planner_->SetStartState(Eigen::Vector3d(current_pose.position.x,
                                            current_pose.position.y,
                                            current_pose.position.z));
    planner_->SetGoalState(Eigen::Vector3d(goal_x,
                                           goal_y,
                                           goal_z));
}

void wxApplicationNode::InjectCurrentPointCloudAsObstacles()
{
    int count = 0;
    for (int i = 0; i < current_point_cloud.points.size(); i++)
    {
        // Center
        Eigen::Vector3d cloud_point(current_point_cloud.points.at(i).x, 
                                    current_point_cloud.points.at(i).y, 
                                    current_point_cloud.points.at(i).z);
        GetPlanner()->statespace_->GetObstacleGrid().InsertOccupiedMeasurement(cloud_point);
        count++;
        
        // Neighbouring Cells as well?
        bool extend_space_around_point = true;
        double delta = 1.0; // [m] (?)
        if(extend_space_around_point)
        {
            for(int x_i = -1; x_i < 2; x_i++)
            {
                for(int y_i = -1; y_i < 2; y_i++)
                {
                    for(int z_i = -1; z_i < 2; z_i++)
                    {
                        // Skip the center point
                        if( x_i != 0 || y_i != 0 || z_i != 0 )
                        {
                            Eigen::Vector3d delta_cloud_point(current_point_cloud.points.at(i).x + delta * x_i, 
                                                              current_point_cloud.points.at(i).y + delta * y_i, 
                                                              current_point_cloud.points.at(i).z + delta * z_i);
                            GetPlanner()->statespace_->GetObstacleGrid().InsertOccupiedMeasurement(delta_cloud_point);
                            count++;
                        }
                    }
                }
            }
        }
    }
}

void wxApplicationNode::UpdateDrawnCanvas(wxIdleEvent &event)
{
    if(ros::ok())
    {
        ros::spinOnce();
    }
    if(external_trigger_subscriber_pose)
    {
        external_trigger_subscriber_pose = false;
        PoseCallback();
    }
    if(external_trigger_subscriber_pcl)
    {
        external_trigger_subscriber_pcl = false;
        PointCloudCallback();
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
