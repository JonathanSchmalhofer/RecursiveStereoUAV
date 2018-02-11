#pragma once

#include <Eigen/Dense>
#include <StateSpace.hpp>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and no obstacles.
 */
class PlaneStateSpace : public StateSpace {
public:
    PlaneStateSpace(double width, double height, double depth)
        : _width(width), _height(height), _depth(depth) {}

    Eigen::Vector3d randomState() const {
        return Eigen::Vector3d(drand48() * width(), drand48() * height(), drand48() * depth());
    }

    Eigen::Vector3d intermediateState(const Eigen::Vector3d& source,
                                  const Eigen::Vector3d& target,
                                  double stepSize) const {
        Eigen::Vector3d delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        Eigen::Vector3d val = source + delta * stepSize;
        return val;
    }

    double distance(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
        Eigen::Vector3d delta = from - to;
        return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2) + powf(delta.z(), 2));
    }

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const Eigen::Vector3d& pt) const {
        return pt.x() >= 0 && pt.y() >= 0 && pt.z() >= 0 && pt.x() < width() &&
               pt.y() < height() && pt.z() < depth();
    }

    double width() const { return _width; }
    double height() const { return _height; }
    double depth() const { return _depth; }

private:
    double _width, _height, _depth;
};

}  // namespace RRT
