#include "wxApplication.hpp"

wxBEGIN_EVENT_TABLE(wxApplicationNode, wxApp)
    EVT_IDLE(wxApplicationNode::DoUpdate)
wxEND_EVENT_TABLE()
wxBEGIN_EVENT_TABLE(RRTGLCanvas, wxGLCanvas)
    EVT_PAINT(RRTGLCanvas::OnPaint)
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
                 wxFULL_REPAINT_ON_RESIZE)
{
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

    RRTGLContext& canvas = wxGetApp().GetContext(this);
    glViewport(0, 0, ClientSize.x, ClientSize.y);

    // Render the graphics and swap the buffers.
    GLboolean quadStereoSupported;
    glGetBooleanv( GL_STEREO, &quadStereoSupported);
    if ( quadStereoSupported )
    {
        glDrawBuffer( GL_BACK_LEFT );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.47f, 0.53f, -0.5f, 0.5f, 1.0f, 3.0f);
        canvas.DrawNow();
        CheckGLError();
        glDrawBuffer( GL_BACK_RIGHT );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(-0.53f, 0.47f, -0.5f, 0.5f, 1.0f, 3.0f);
        canvas.DrawNow();
        CheckGLError();
    }
    else
    {
        canvas.DrawNow();
    }
    SwapBuffers();
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
    : wxGLContext(canvas)
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

    // create the textures to use for cube sides: they will be reused by all
    // canvases (which is probably not critical in the case of simple textures
    // we use here but could be really important for a real application where
    // each texture could take many megabytes)
    glGenTextures(WXSIZEOF(textures_), textures_);

    for ( unsigned i = 0; i < WXSIZEOF(textures_); i++ )
    {
        glBindTexture(GL_TEXTURE_2D, textures_[i]);

        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        const wxImage img(DrawDice(256, i + 1));

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.GetWidth(), img.GetHeight(),
                     0, GL_RGB, GL_UNSIGNED_BYTE, img.GetData());
    }

    CheckGLError();
}

void RRTGLContext::DrawNow()
{    
    
    float xangle = 0;
    float yangle = 0;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.0f);
    glRotatef(xangle, 1.0f, 0.0f, 0.0f);
    glRotatef(yangle, 0.0f, 1.0f, 0.0f);
    
    
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
    
    /*

    // draw six faces of a cube of size 1 centered at (0, 0, 0)
    glBindTexture(GL_TEXTURE_2D, textures_[0]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 0.0f, 1.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f,-0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f,-0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, textures_[1]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 0.0f,-1.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f, 0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f, 0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f,-0.5f,-0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, textures_[2]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f, 1.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f, 0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f, 0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f, 0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, textures_[3]);
    glBegin(GL_QUADS);
        glNormal3f( 0.0f,-1.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f,-0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f,-0.5f, 0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, textures_[4]);
    glBegin(GL_QUADS);
        glNormal3f( 1.0f, 0.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f( 0.5f, 0.5f, 0.5f);
        glTexCoord2f(1, 0); glVertex3f( 0.5f,-0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f( 0.5f,-0.5f,-0.5f);
        glTexCoord2f(0, 1); glVertex3f( 0.5f, 0.5f,-0.5f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, textures_[5]);
    glBegin(GL_QUADS);
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glTexCoord2f(0, 0); glVertex3f(-0.5f,-0.5f,-0.5f);
        glTexCoord2f(1, 0); glVertex3f(-0.5f,-0.5f, 0.5f);
        glTexCoord2f(1, 1); glVertex3f(-0.5f, 0.5f, 0.5f);
        glTexCoord2f(0, 1); glVertex3f(-0.5f, 0.5f,-0.5f);
    glEnd();
    
    */

    glFlush();

    CheckGLError();
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

// function to draw the texture for cube faces
static wxImage DrawDice(int size, unsigned num)
{
    wxASSERT_MSG( num >= 1 && num <= 6, wxT("invalid dice index") );

    const int dot = size/16;        // radius of a single dot
    const int gap = 5*size/32;      // gap between dots

    wxBitmap bmp(size, size);
    wxMemoryDC dc;
    dc.SelectObject(bmp);
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();
    dc.SetBrush(*wxBLACK_BRUSH);

    // the upper left and lower right points
    if ( num != 1 )
    {
        dc.DrawCircle(gap + dot, gap + dot, dot);
        dc.DrawCircle(size - gap - dot, size - gap - dot, dot);
    }

    // draw the central point for odd dices
    if ( num % 2 )
    {
        dc.DrawCircle(size/2, size/2, dot);
    }

    // the upper right and lower left points
    if ( num > 3 )
    {
        dc.DrawCircle(size - gap - dot, gap + dot, dot);
        dc.DrawCircle(gap + dot, size - gap - dot, dot);
    }

    // finally those 2 are only for the last dice
    if ( num == 6 )
    {
        dc.DrawCircle(gap + dot, size/2, dot);
        dc.DrawCircle(size - gap - dot, size/2, dot);
    }

    dc.SelectObject(wxNullBitmap);

    return bmp.ConvertToImage();
}

wxString glGetwxString(GLenum name)
{    
    const GLubyte *v = glGetString(name);
    if ( v == 0 )
    {
        // The error is not important. It is GL_INVALID_ENUM.
        // We just want to clear the error stack.
        glGetError();

        return wxString();
    }

    return wxString((const char*)v);
}
