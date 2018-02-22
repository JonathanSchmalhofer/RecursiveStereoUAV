#ifndef ROBOJACKETS_3D_TRAJECTORY_SPACE_STATE_SPACE_H
#define ROBOJACKETS_3D_TRAJECTORY_SPACE_STATE_SPACE_H

#include <Eigen/Dense>
#include "state_space.h"

namespace RRT
{

/// @brief A 3d space with continuous states and no obstacles.
class SpaceStateSpace
    : public StateSpace
{
public:
    SpaceStateSpace(double width,
                    double height,
                    double depth)
        : width_(width), height_(height), depth_(depth) {}

    Eigen::Vector3d GetRandomState() const
    {
        return Eigen::Vector3d(drand48() * GetWidth(),
                               drand48() * GetHeight(),
                               drand48() * GetDepth());
    }

    Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                         const Eigen::Vector3d& target,
                                         double step_size) const
    {
        Eigen::Vector3d delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        Eigen::Vector3d val = source + delta * step_size;
        return val;
    }

    double GetDistance(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
    {
        Eigen::Vector3d delta = from - to;
        return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2) + powf(delta.z(), 2));
    }


    /// @brief Returns a boolean indicating whether the given point is within bounds.
    bool IsStateValid(const Eigen::Vector3d& point) const
    {
        return point.x() >= 0 && point.y() >= 0 && point.z() >= 0 &&
               point.x() < GetWidth() && point.y() < GetHeight() && point.z() < GetDepth();
    }

    double GetWidth() const { return width_; }
    double GetHeight() const { return height_; }
    double GetDepth() const { return depth_; }

private:
    double width_, height_, depth_;
};

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_SPACE_STATE_SPACE_H
