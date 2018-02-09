#pragma once

#include "PlannerWrapper.hpp"

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

static void CheckGLError();

// function to draw the texture for cube faces
static wxImage DrawDice(int size, unsigned num);

wxString glGetwxString(GLenum name);

////////////////////////////////////////////////////////////////////////////////////

// the rendering context used by all GL canvases
class RRTGLContext : public wxGLContext
{
public:
    RRTGLContext(wxGLCanvas *canvas);

    // render the cube showing it at given angles
    void DrawNow();

private:
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

class RRTGLCanvas : public wxGLCanvas
{
public:
    RRTGLCanvas(wxWindow *parent, int *attribList = NULL);

    void ClearTrees();
    void AddTree(TreeToDraw tree);
    void DrawNow();
    void ResetView();
    void Spin(double angle_x, double angle_y, double angle_z);

private:
    /// @brief Rotation angle around x axis for view.
    double view_rotation_angle_x_;

    /// @brief Rotation angle around y axis for view.
    double view_rotation_angle_y_;

    /// @brief Rotation angle around z axis for view.
    double view_rotation_angle_z_;

    /// @brief Translation along x axis for view.
    double view_translation_x_;

    /// @brief Translation along y axis for view.
    double view_translation_y_;

    /// @brief Translation along z axis for view.
    double view_translation_z_;

    std::vector<Eigen::Vector2d> points_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines_;    
    std::vector<TreeToDraw> trees_;

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

private:
    void ExtractPointsAndLinesFromTreeForCanvas();

    PlannerWrapper *planner_;
    // the GL context we use for all our mono rendering windows
    RRTGLContext *context_;
    RRTFrame *frame_;

    wxDECLARE_EVENT_TABLE();
};
