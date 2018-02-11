#ifndef ROBOJACKETS_3D_TRAJECTORY_3DSPACE_H
#define ROBOJACKETS_3D_TRAJECTORY_3DSPACE_H

#include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include "StateSpace.h"
#include "Tree.h"

namespace RRT {

const int dimensions = 3;
 
/**
 * Hash function for Eigen::Vector3d
 */
static size_t hash3d(Eigen::Vector3d state) {
    size_t seed = 0;
    boost::hash_combine(seed, state.x());
    boost::hash_combine(seed, state.y());
    boost::hash_combine(seed, state.z());
    return seed;
}

/**
 * This creates an instance of an RRT Tree with the callbacks
 * configured from the given parameters.
 *
 * @param w, h The dimensions of the 3d space.  These are used when
 * picking random points for the Tree to move towards.

 * @param goal The point representing the goal that the tree is
 * trying to find a path to
 *
 * @param step The fixed step size that the tree uses.  This is the
 * maximum distance between nodes unless adaptive stepsize control is utilized.
 *
 * @return An RRT::Tree with its callbacks and parameters configured.
 * You'll probably want to override the transitionValidator callback
 * if your 3d space has any obstacles.
 */
std::shared_ptr<RRT::Tree<Eigen::Vector3d>> TreeFor3dSpace(
    std::shared_ptr<StateSpace> stateSpace,
    Eigen::Vector3d goal, double step);

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_3DSPACE_H
