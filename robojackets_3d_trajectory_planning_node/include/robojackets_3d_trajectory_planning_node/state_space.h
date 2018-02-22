#ifndef ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_STATE_SPACE_H
#define ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_STATE_SPACE_H

namespace RRT
{

/// A state space represents the set of possible states for a planning problem.
/// This includes the obstacles that may be present and what state transitions
/// are valid.
/// This class is abstract and must be subclassed in order to provide actual
class StateSpace
{
public:
    StateSpace(){};
    virtual ~StateSpace(){};

    /// @brief Generate a random state within the bounds of the state space.
    /// 
    /// @return A random state
    virtual Eigen::Vector3d GetRandomState() const = 0;

    /// @brief Finds a state in the direction of @target from @source.state().
    ///
    /// @detail This new state will potentially be added to the tree.  No need to do
    /// any validation on the state before returning, the tree will handle
    /// that.
    virtual Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                                 const Eigen::Vector3d& target,
                                                 double step_size) const = 0;

    /// @brief An overloaded version designed for use in adaptive stepsize control.
    ///
    /// @param source The node in the tree to extend from
    /// @param target The point in the space to extend to
    /// @param min_step_size The minimum allowable stepsize the intermediate state
    /// will be extended from source
    /// @param max_step_size The maximum allowable stepsize the intermediate state
    /// will be extended from source
    /// 
    /// @return A state in the direction of @target from @source.state()
    virtual Eigen::Vector3d GetIntermediateState(const Eigen::Vector3d& source,
                                                 const Eigen::Vector3d& target,
                                                 double min_step_size,
                                                 double max_step_size) const = 0;

    /// @brief Calculate the distance between two states
    /// 
    /// @param from Start state
    /// @param to End state
    /// 
    /// @return The distance between the states
    virtual double GetDistance(const Eigen::Vector3d& from,
                               const Eigen::Vector3d& to) const = 0;

    /// @brief Check if a state is within bounds and obstacle-free
    /// 
    /// @param state The state to check
    /// @return A boolean indicating validity
    virtual bool IsStateValid(const Eigen::Vector3d& state) const = 0;

    /// @brief Check motion validity from one state to another
    /// @details Returns a boolean indicating whether or not a direct motion from
    /// one state to another is valid.
    /// 
    /// @param from The start state
    /// @param to The destination state
    /// 
    /// @return A boolean indicating validity
    virtual bool IsTransitionValid(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const = 0;

protected:
    double min_step_size_;
    double max_step_size_;
};

}  // namespace RRT

#endif // ROBOJACKETS_3D_TRAJECTORY_PLANNING_NODE_STATE_SPACE_H
