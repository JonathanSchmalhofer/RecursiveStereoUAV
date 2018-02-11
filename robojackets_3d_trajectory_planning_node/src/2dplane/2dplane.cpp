
#include <math.h>
#include <memory>
#include <functional>
#include <2dplane/2dplane.hpp>

using namespace Eigen;
using namespace RRT;
using namespace std;

shared_ptr<Tree<Vector3d>> RRT::TreeFor3dPlane(
    shared_ptr<StateSpace> stateSpace, Vector3d goal,
    double step) {
    shared_ptr<Tree<Vector3d>> rrt =
        make_shared<Tree<Vector3d>>(stateSpace, dimensions);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
