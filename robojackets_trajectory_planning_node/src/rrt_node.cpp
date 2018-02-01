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


class PlannerWrapper
{
public:
    PlannerWrapper();
    void step();

    std::shared_ptr<RRT::GridStateSpace> statespace_;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector2d>> birrt_;

    Eigen::Vector2d start_velocity_;
    Eigen::Vector2d goal_velocity_;
    std::vector<Eigen::Vector2d> previous_solution_;
};

PlannerWrapper::PlannerWrapper()
{
    Eigen::Vector2d size(800, 600);
    statespace_ = std::make_shared<RRT::GridStateSpace>(size.x(), size.y(), 40, 30);
    birrt_ = std::make_unique<RRT::BiRRT<Eigen::Vector2d>>(statespace_, 2);

    //  setup birrt
    birrt_->setStartState(Eigen::Vector2d(50,50));
    birrt_->setGoalState(Eigen::Vector2d(750,550));
    birrt_->setMaxStepSize(100);
    birrt_->setStepSize(10);
    birrt_->setMaxIterations(1000);
    birrt_->setASCEnabled(true);
    birrt_->setGoalBias(0);
    birrt_->setWaypointBias(0);
    birrt_->setGoalMaxDist(5);
    

    start_velocity_ = Eigen::Vector2d(3, 0);
    goal_velocity_ = Eigen::Vector2d(0, 3);
}

void PlannerWrapper::step()
{
    int number_iterations = 5;

    for (int i = 0; i < number_iterations; i++)
    {
        birrt_->grow();
    }

    ROS_INFO("#%d with size() = %d", birrt_->iterationCount(), birrt_->getPath().size());

    // store solution
    previous_solution_.clear();
    if (birrt_->startSolutionNode() != nullptr)
    {
        ROS_INFO("   Found a solution");
        previous_solution_ = birrt_->getPath();
        RRT::SmoothPath(previous_solution_, *statespace_);
    }
}

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
    }
}

void BasicDrawPane::ClearTrees()
{
    trees_.clear();
}

void BasicDrawPane::AddTree(TreeToDraw tree)
{
    trees_.push_back(tree);
}

////////////////////////////////////////////////////////////////////////////////////

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()

wxBEGIN_EVENT_TABLE(BasicDrawPane, wxPanel)
    EVT_PAINT(BasicDrawPane::paintEvent)
wxEND_EVENT_TABLE()

IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

bool wxApplicationNode::OnInit()
{
    planner_ = new PlannerWrapper();

    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
    frame_ = new wxFrame((wxFrame *)NULL, -1,  wxT("BiRRT"), wxPoint(50,50), wxSize(800,600));
    
    draw_pane_ = new BasicDrawPane( (wxFrame*) frame_ );
    sizer->Add(draw_pane_, 1, wxEXPAND);
 
    frame_->SetSizer(sizer);
    frame_->SetAutoLayout(true);
 
    frame_->Show(true);
    return true;
}

void wxApplicationNode::DoUpdate(wxIdleEvent &event)
{
    if(ros::ok())
    {
        planner_->step();
        ros::spinOnce();
	    ros::Duration(1).sleep();
    }

    ExtractPointsAndLinesFromTreeForDrawPane();
    draw_pane_->paintNow();

    if(IsMainLoopRunning())
    {
        event.RequestMore();
    }
}

void wxApplicationNode::ExtractPointsAndLinesFromTreeForDrawPane()
{
    draw_pane_->ClearTrees();

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

    //draw_pane_->trees_.push_back(*start_tree);
    draw_pane_->AddTree(*start_tree);
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

    //draw_pane_->trees_.push_back(*goal_tree);
    draw_pane_->AddTree(*goal_tree);
    delete goal_tree;
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
