#include "wxApplication.hpp"

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()

wxBEGIN_EVENT_TABLE(BasicDrawPane, wxPanel)
    EVT_PAINT(BasicDrawPane::paintEvent)
wxEND_EVENT_TABLE()

IMPLEMENT_APP_NO_MAIN(wxApplicationNode);
IMPLEMENT_WX_THEME_SUPPORT;

////////////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
// BasicDrawPane
// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------
// wxApplicationNode
// ----------------------------------------------------------------------------

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
