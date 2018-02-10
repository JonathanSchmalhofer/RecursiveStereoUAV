#pragma once

#include "PlannerWrapper.hpp"

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

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

class BasicDrawPane
    : public wxPanel
{
 
public:
    BasicDrawPane(wxFrame* parent);
 
    void paintEvent(wxPaintEvent & evt);
    void paintNow();
 
    void render(wxDC& dc);
    
    void ClearTrees();
    void AddTree(TreeToDraw tree);
private: 
    std::vector<Eigen::Vector2d> points_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines_;    
    std::vector<TreeToDraw> trees_;

    wxDECLARE_EVENT_TABLE();
};

class wxApplicationNode
    : public wxApp
{
public:
    virtual bool OnInit();
    void DoUpdate(wxIdleEvent &event);
 
private:
    void ExtractPointsAndLinesFromTreeForDrawPane();

    wxFrame *frame_;
    BasicDrawPane *draw_pane_;
    PlannerWrapper *planner_;
 
    wxDECLARE_EVENT_TABLE();
};
