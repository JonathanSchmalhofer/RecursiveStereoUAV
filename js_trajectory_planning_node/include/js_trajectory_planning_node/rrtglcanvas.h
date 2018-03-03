#ifndef JS_TRAJECTORY_PLANNING_NODE_RRTGLCANVAS_H
#define JS_TRAJECTORY_PLANNING_NODE_RRTGLCANVAS_H

#include "tree_to_draw.h"
#include "constants.h"

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

#endif // JS_TRAJECTORY_PLANNING_NODE_RRTGLCANVAS_H
