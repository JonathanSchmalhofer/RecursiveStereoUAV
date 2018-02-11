#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_PATH_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_PATH_H

#include "state_space.h"
#include <vector>

namespace RRT {

/**
 * @brief Reduce the vector to @maxSize length
 * @details We do this by sampling to evenly distribute deletions
 *
 * @param states     The vector of T values to sample
 * @param maxSize Max length of the resulting vector
 */
void DownSampleVector(std::vector<Eigen::Vector3d>& states, size_t maxSize) {
    if (states.size() > maxSize) {
        int toDelete = states.size() - maxSize;
        double spacing = (double)states.size() / (double)toDelete;
        double i = 0.0;
        while (toDelete) {
            toDelete--;
            states.erase(states.begin() + (int)(i + 0.5));
            i += spacing - 1.0;
        }
    }
}

/**
 * @brief Deletes waypoints from @pts
 *
 * @param pts A vector of states that constitutes the path
 * @param transitionValidator A function that returns a boolean indicating
 *        whether or not a straight connection exists between the two given
 * states
 */
void SmoothPath(std::vector<Eigen::Vector3d>& pts, const StateSpace& stateSpace) {
    int span = 2;
    while (span < pts.size()) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            if (stateSpace.transitionValid(pts[i], pts[i + span])) {
                for (int x = 1; x < span; x++) {
                    pts.erase(pts.begin() + i + 1);
                }
                changed = true;
            }
        }

        if (!changed) span++;
    }
}

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_PATH_H

