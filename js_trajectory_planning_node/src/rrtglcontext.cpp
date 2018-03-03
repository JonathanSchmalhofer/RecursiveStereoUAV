#include "js_trajectory_planning_node/rrtglcontext.h"
#include "js_trajectory_planning_node/constants.h"
#include "js_trajectory_planning_node/helpers.h"

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
      view_look_at_point_z_(khalf_depth)
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

        glBegin(GL_POINTS); // starts drawing of points
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
        glEnd(); // end drawing of points

        glColor3f(0.0f,0.0f,1.0f); //blue color as default

        glBegin(GL_LINES); // starts drawing of lines
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
        glEnd(); // end drawing of lines
    }

    glPointSize(20.0f); //set point size to 20 pixels
    
    // Start Point
    glColor3f(0.0f,1.0f,0.0f); // GREEN for start point
    glBegin(GL_POINTS); // starts drawing of start point
        glVertex3f(GetStartPoint().x(),
                   GetStartPoint().y(),
                   GetStartPoint().z());
    glEnd(); // end drawing of start point
    
    // Goal Point
    glColor3f(1.0f,0.65f,0.0f); // ORANGE for goal point
    glBegin(GL_POINTS); // starts drawing of goal point
        glVertex3f(GetGoalPoint().x(),
                   GetGoalPoint().y(),
                   GetGoalPoint().z());
    glEnd(); // end drawing of goal point

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

Eigen::Vector3d RRTGLContext::GetStartPoint()
{
    return start_point_;
}

Eigen::Vector3d RRTGLContext::GetGoalPoint()
{
    return goal_point_;
}

void RRTGLContext::SetStartPoint(Eigen::Vector3d start_point)
{
    start_point_ = start_point;
}

void RRTGLContext::SetGoalPoint(Eigen::Vector3d goal_point)
{
    goal_point_ = goal_point;
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
