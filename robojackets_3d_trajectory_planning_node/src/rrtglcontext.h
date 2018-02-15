#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTGLCONTEXT_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTGLCONTEXT_H

#include "tree_to_draw.h"

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

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_RRTGLCONTEXT_H
