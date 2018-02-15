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

void CheckGLError()
{
    GLenum last_error = GL_NO_ERROR;

    for ( ;; )
    {
        GLenum error = glGetError();
        if ( error == GL_NO_ERROR )
            return;

        // normally the error is reset by the call to glGetError() but if
        // glGetError() itself returns an error, we risk looping forever here
        // so check that we get a different error than the last time
        if ( error == last_error )
        {
            wxLogError(wxT("OpenGL error state couldn't be reset."));
            return;
        }

        last_error = error;

        wxLogError(wxT("OpenGL error %d"), error);
    }
}
