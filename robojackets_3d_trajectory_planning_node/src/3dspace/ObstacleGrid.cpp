#include "ObstacleGrid.h"
#include <stdlib.h>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace RRT {

ObstacleGrid::ObstacleGrid(double width, double height, double depth, int discretizedWidth,
                           int discretizedHeight, int discretizedDepth) {
    _width = width;
    _height = height;
    _depth = depth;
    _discretizedWidth = discretizedWidth;
    _discretizedHeight = discretizedHeight;
    _discretizedDepth = discretizedDepth;

    _obstacles =
        (bool*)malloc(sizeof(bool) * discretizedWidth * discretizedHeight * discretizedDepth);

    clear();
}

ObstacleGrid::~ObstacleGrid() { free(_obstacles); }

Vector3i ObstacleGrid::gridSquareForLocation(const Vector3d& loc) const {
    return Vector3i(loc.x() / width() * discretizedWidth(),
                    loc.y() / height() * discretizedHeight(),
                    loc.z() / height() * discretizedDepth());
}

double ObstacleGrid::nearestObstacleDist(const Vector3d& state,
                                        double maxDist) const {
    // x and y are the indices of the cell that state is located in
    double x = (state.x() / (_width / _discretizedWidth));
    double y = (state.y() / (_height / _discretizedHeight));
    double z = (state.z() / (_depth / _discretizedDepth));
    int xSearchRad = maxDist * _discretizedWidth / _width;
    int ySearchRad = maxDist * _discretizedHeight / _height;
    int zSearchRad = maxDist * _discretizedDepth / _depth;
    // here we loop through the cells around (x,y,z) to find the minimum distance
    // of
    // the point to the nearest obstacle
    for (int i = x - xSearchRad; i <= x + xSearchRad; i++) {
        for (int j = y - ySearchRad; j <= y + ySearchRad; j++) {
            for (int k = z - zSearchRad; k <= z + zSearchRad; k++) {
                bool obs = obstacleAt(i, j, k);
                if (obs) {
                    double xDist = (x - i) * _width / _discretizedWidth;
                    double yDist = (y - j) * _height / _discretizedHeight;
                    double zDist = (z - k) * _depth / _discretizedDepth;
                    double dist = sqrtf(powf(xDist, 2) + powf(yDist, 2) + powf(zDist, 2));
                    if (dist < maxDist) {
                        maxDist = dist;
                    }
                }
            }
        }
    }

    // the boundaries of the grid count as obstacles
    maxDist = std::min(maxDist, state.x());             // left boundary
    maxDist = std::min(maxDist, width() - state.x());   // right boundary
    maxDist = std::min(maxDist, state.y());             // top boundary
    maxDist = std::min(maxDist, height() - state.y());  // bottom boundary
    maxDist = std::min(maxDist, state.z());             // front boundary
    maxDist = std::min(maxDist, depth() - state.z());   // back boundary

    return maxDist;
}

void ObstacleGrid::clear() {
    for (int x = 0; x < discretizedWidth(); x++) {
        for (int y = 0; y < discretizedHeight(); y++) {
            for (int z = 0; z < discretizedDepth(); z++) {
                obstacleAt(x, y, z) = false;
            }
        }
    }
}

bool& ObstacleGrid::obstacleAt(int x, int y, int z) {
    return _obstacles[x + _discretizedWidth * y + _discretizedDepth * z];
}

bool ObstacleGrid::obstacleAt(int x, int y, int z) const {
    return _obstacles[x + _discretizedWidth * y + _discretizedDepth * z];
}

bool& ObstacleGrid::obstacleAt(const Vector3i& gridLoc) {
    return obstacleAt(gridLoc.x(), gridLoc.y(), gridLoc.z());
}

bool ObstacleGrid::obstacleAt(const Vector3i& gridLoc) const {
    return obstacleAt(gridLoc.x(), gridLoc.y(), gridLoc.z());
}

int ObstacleGrid::discretizedWidth() const { return _discretizedWidth; }

int ObstacleGrid::discretizedHeight() const { return _discretizedHeight; }

int ObstacleGrid::discretizedDepth() const { return _discretizedDepth; }

double ObstacleGrid::width() const { return _width; }

double ObstacleGrid::height() const { return _height; }

double ObstacleGrid::depth() const { return _depth; }

}  // namespace RRT
