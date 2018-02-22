#include <math.h>
#include <memory>
#include <functional>
#include "robojackets_3d_trajectory_planning_node/3dspace/3dspace.h"

using namespace Eigen;
using namespace RRT;
using namespace std;

shared_ptr<Tree<Vector3d>> RRT::GetTreeFor3dSpace(
    shared_ptr<StateSpace> state_space,
    Vector3d goal,
    double step_size)
{
    shared_ptr<Tree<Vector3d>> rrt =
        make_shared<Tree<Vector3d>>(state_space, dimensions);

    rrt->SetStepSize(step_size);

    rrt->SetGoalState(goal);

    return rrt;
}
