#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H

#include "planner_wrapper.h"

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

const int khalf_width = 400;
const int khalf_height = 400;
const int khalf_depth = 400;

const double kdelta_angle = 5; // [deg]
const double kdelta_translation = 10; // [-]

static void CheckGLError();

////////////////////////////////////////////////////////////////////////////////////

typedef Eigen::Vector3d           Point;
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

// the rendering context used by all GL canvases
class RRTGLContext : public wxGLContext
{
public:
    RRTGLContext(wxGLCanvas *canvas);

    void ClearTrees();
    void AddTree(TreeToDraw tree);

    /// @brief Get the start point
    Eigen::Vector3d GetStartPoint();

    /// @brief Get the goal point
    Eigen::Vector3d GetGoalPoint();

    /// @brief Set the start point
    void SetStartPoint(Eigen::Vector3d start_point);

    /// @brief Set the goal point
    void SetGoalPoint(Eigen::Vector3d goal_point);

    // render the cube showing it at given angles
    void DrawNow();

    /// @brief Get azimuth angle.
    double GetAzimuth();

    /// @brief Get elevation angle.
    double GetElevation();

    /// @brief Get radius.
    double GetRadius();

    /// @brief Get the look-at-point x coordinate.
    double GetLookAtPointX();

    /// @brief Get the look-at-point y coordinate.
    double GetLookAtPointY();

    /// @brief Get the look-at-point z coordinate.
    double GetLookAtPointZ();

    /// @brief Set azimuth angle.
    void SetAzimuth(double azimuth);

    /// @brief Set elevation angle.
    void SetElevation(double elevation);

    /// @brief Set radius.
    void SetRadius(double radius);

    /// @brief Set the look-at-point x coordinate.
    void SetLookAtPointX(double look_at_x);

    /// @brief Set the look-at-point y coordinate.
    void SetLookAtPointY(double look_at_y);

    /// @brief Set the look-at-point z coordinate.
    void SetLookAtPointZ(double look_at_z);

private:
    std::vector<Eigen::Vector3d> points_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> lines_;
    std::vector<TreeToDraw> trees_;
    Eigen::Vector3d start_point_;
    Eigen::Vector3d goal_point_;

    /// @brief Azimuth angle.
    double view_azimuth_;

    /// @brief Elevation angle.
    double view_elevation_;

    /// @brief Radius.
    double radius_;

    /// @brief Look-at-point x coordinate.
    double view_look_at_point_x_;

    /// @brief Look-at-point y coordinate.
    double view_look_at_point_y_;

    /// @brief Look-at-point z coordinate.
    double view_look_at_point_z_;
};


////////////////////////////////////////////////////////////////////////////////////

class RRTGLCanvas : public wxGLCanvas
{
public:
    RRTGLCanvas(wxWindow *parent, int *attribList = NULL);

    void DrawNow();
    void ResetView();
    void Spin(double azimuth, double elevation);
    void Translate(double translate_x, double translate_y, double translate_z);
    void ChangeRadius(double delta_radius);

    /// @brief Get tree.
    std::vector<TreeToDraw> GetTree();

    /// @brief Get azimuth angle.
    double GetAzimuth();

    /// @brief Get elevation angle.
    double GetElevation();

    /// @brief Get radius.
    double GetRadius();

    /// @brief Get the look-at-point x coordinate.
    double GetLookAtPointX();

    /// @brief Get the look-at-point y coordinate.
    double GetLookAtPointY();

    /// @brief Get the look-at-point z coordinate.
    double GetLookAtPointZ();

    /// @brief Set azimuth angle.
    void SetAzimuth(double azimuth);

    /// @brief Set elevation angle.
    void SetElevation(double elevation);

    /// @brief Set radius.
    void SetRadius(double radius);

    /// @brief Set the look-at-point x coordinate.
    void SetLookAtPointX(double look_at_x);

    /// @brief Set the look-at-point y coordinate.
    void SetLookAtPointY(double look_at_y);

    /// @brief Set the look-at-point z coordinate.
    void SetLookAtPointZ(double look_at_z);

private:
    /// @brief Azimuth angle.
    double view_azimuth_;

    /// @brief Elevation angle.
    double view_elevation_;

    /// @brief Radius.
    double radius_;

    /// @brief Look-at-point x coordinate.
    double view_look_at_point_x_;

    /// @brief Look-at-point y coordinate.
    double view_look_at_point_y_;

    /// @brief Look-at-point z coordinate.
    double view_look_at_point_z_;

    void OnPaint(wxPaintEvent& event);
    void OnKeyDown(wxKeyEvent& event);

    wxDECLARE_EVENT_TABLE();
};

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



class wxApplicationNode
    : public wxApp
{
public:
    wxApplicationNode() { context_ = NULL;}

    // Returns the shared context used by all frames and sets it as current for
    // the given canvas.
    RRTGLContext& GetContext(wxGLCanvas *canvas);

    // virtual wxApp methods
    virtual bool OnInit();
    virtual int OnExit();

    void DoUpdate(wxIdleEvent &event);

    RRTFrame* GetFrame();
    PlannerWrapper* GetPlanner();

private:
    void ExtractStartAndGoalStateForContext();
    void ExtractPointsAndLinesFromTreeForContext();

    PlannerWrapper *planner_;
    // the GL context we use for all our mono rendering windows
    RRTGLContext *context_;
    RRTFrame *frame_;

    wxDECLARE_EVENT_TABLE();
};

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_WXAPPLICATIONNODE_H
