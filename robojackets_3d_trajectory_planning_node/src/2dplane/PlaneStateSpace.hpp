#pragma once

#include <Eigen/Dense>
#include <StateSpace.hpp>

namespace RRT {

/**
 * @brief A 2d plane with continuous states and no obstacles.
 */
class PlaneStateSpace : public StateSpace {
public:
    PlaneStateSpace(double width, double height)
        : _width(width), _height(height) {}

    Eigen::Vector2d randomState() const {
        return Eigen::Vector2d(drand48() * width(), drand48() * height());
    }

    Eigen::Vector2d intermediateState(const Eigen::Vector2d& source,
                                  const Eigen::Vector2d& target,
                                  double stepSize) const {
        Eigen::Vector2d delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        Eigen::Vector2d val = source + delta * stepSize;
        return val;
    }

    double distance(const Eigen::Vector2d& from, const Eigen::Vector2d& to) const {
        Eigen::Vector2d delta = from - to;
        return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
    }

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const Eigen::Vector2d& pt) const {
        return pt.x() >= 0 && pt.y() >= 0 && pt.x() < width() &&
               pt.y() < height();
    }

    double width() const { return _width; }
    double height() const { return _height; }

private:
    double _width, _height;
};

}  // namespace RRT
