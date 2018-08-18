#ifndef JS_TRAJECTORY_PLANNING_NODE_BIRRT_H
#define JS_TRAJECTORY_PLANNING_NODE_BIRRT_H

#include <limits.h>
#include "tree.h"

namespace RRT
{
    
/**
 * @brief Bi-directional RRT
 * @details It is often preferable to use two RRTs when searching the state
 *     space with one rooted at the source and one rooted at the goal.  When the
 *     two trees intersect, a solution has been found.
 */
template <typename T>
class BiRRT
{
public:
    BiRRT(std::shared_ptr<StateSpace> stateSpace, int dimensions)
        : start_tree_(stateSpace, dimensions),
          goal_tree_(stateSpace, dimensions)
    {
        min_iterations_ = 0;
        Reset();
    }

    void Reset()
    {
        start_tree_.Reset(true);
        goal_tree_.Reset(true);
        start_tree_.ClearWaypoints();
        goal_tree_.ClearWaypoints();

        count_iterations_ = 0;

        start_solution_node_ = nullptr;
        goal_solution_node_ = nullptr;
        solution_length_ = INT_MAX;
    }

    const Tree<T>& GetStartTree() const { return start_tree_; }
    const Tree<T>& GetGoalTree() const { return goal_tree_; }

    bool IsAdaptiveScalingEnable() const { return start_tree_.IsAdaptiveScalingEnable(); }
    void SetAdaptiveScalingEnable(bool checked)
    {
        start_tree_.SetAdaptiveScalingEnable(checked);
        goal_tree_.SetAdaptiveScalingEnable(checked);
    }

    double GetGoalBias() const { return start_tree_.GetGoalBias(); }
    void SetGoalBias(double goal_bias)
    {
        start_tree_.SetGoalBias(goal_bias);
        goal_tree_.SetGoalBias(goal_bias);
    }

    int GetMaxIterations() const { return start_tree_.GetMaxIterations(); }
    void SetMaxIterations(int itr)
    {
        start_tree_.SetMaxIterations(itr);
        goal_tree_.SetMaxIterations(itr);
    }

    /**
     * The minimum number of iterations to run.
     *
     * At the default value of zero, the rrt will return the first path it
     * finds. Setting this to a higher value can allow the tree to search for
     * longer in order to find a better path.
     */
    int GetMinIterations() const { return min_iterations_; }
    void SetMinIterations(int itr) { min_iterations_ = itr; }

    double GetWaypointBias() const { return start_tree_.GetWaypointBias(); }
    void SetWaypointBias(double waypoint_bias)
    {
        start_tree_.SetWaypointBias(waypoint_bias);
        goal_tree_.SetWaypointBias(waypoint_bias);
    }

    const std::vector<T>& waypoints() { return start_tree_.waypoints(); }
    void setWaypoints(const std::vector<T>& waypoints)
    {
        start_tree_.setWaypoints(waypoints);
        goal_tree_.setWaypoints(waypoints);
    }

    double GetStepSize() const { return start_tree_.GetStepSize(); }
    void SetStepSize(double step_size)
    {
        start_tree_.SetStepSize(step_size);
        goal_tree_.SetStepSize(step_size);
    }

    double GetMaxStepSize() const { return start_tree_.GetMaxStepSize(); }
    void SetMaxStepSize(double step_size)
    {
        start_tree_.SetMaxStepSize(step_size);
        goal_tree_.SetMaxStepSize(step_size);
    }

    double GetMaxDistanceToGoal() const { return start_tree_.GetMaxDistanceToGoal(); }
    void SetMaxDistanceToGoal(double max_distance)
    {
        start_tree_.SetMaxDistanceToGoal(max_distance);
        goal_tree_.SetMaxDistanceToGoal(max_distance);
    }

    /// @brief Get the shortest path from the start to the goal
    std::vector<T> GetPath()
    {
        std::vector<T> path;
        start_tree_.GetPath(&path, start_solution_node_);
        start_tree_.GetPath(&path, goal_solution_node_, true);
        return path;
    }

     /// @brief
     /// 
     /// @details Attempts to add a new node to each of the two trees.  If
     /// a new solution is found that is shorter than any previous solution, we
     /// store
     /// it instead.
    void Grow()
    {
        int depth;
        const Node<T>* other_node;

        Node<T>* new_start_node = start_tree_.Grow();
        if (new_start_node)
        {
            other_node = FindBestPath(new_start_node->GetState(), goal_tree_, &depth);
            if (other_node && depth + new_start_node->GetDepth() < solution_length_ &&
                goal_tree_.GetStateSpace().IsTransitionValid(new_start_node->GetState(),
                                                       other_node->GetState()))
            {
                start_solution_node_ = new_start_node;
                goal_solution_node_ = other_node;
                solution_length_ = new_start_node->GetDepth() + depth;
            }
        }

        Node<T>* new_goal_node = goal_tree_.Grow();
        if (new_goal_node)
        {
            other_node = FindBestPath(new_goal_node->GetState(), start_tree_, &depth);
            if (other_node && depth + new_goal_node->GetDepth() < solution_length_ &&
                goal_tree_.GetStateSpace().IsTransitionValid(new_goal_node->GetState(),
                                                       other_node->GetState()))
            {
                start_solution_node_ = other_node;
                goal_solution_node_ = new_goal_node;
                solution_length_ = new_goal_node->GetDepth() + depth;
            }
        }

        ++count_iterations_;
    }

    /// @brief Grows the trees until we find a solution or run out of iterations.
    /// 
    /// @return true if a solution is found
    bool Run()
    {
        for (int i = 0; i < start_tree_.GetMaxIterations(); i++)
        {
            Grow();
            if (start_solution_node_ != nullptr && i >= GetMinIterations())
                return true;
        }
        return false;
    }

    void SetStartState(const T& start)
    {
        start_tree_.SetStartState(start);
        goal_tree_.SetGoalState(start);
    }
    const T& GetStartState() const { return start_tree_.GetStartState(); }

    void SetGoalState(const T& goal)
    {
        start_tree_.SetGoalState(goal);
        goal_tree_.SetStartState(goal);
    }
    const T& GetGoalState() const { return start_tree_.GetGoalState(); }

    const Node<T>* GetStartSolutionNode() { return start_solution_node_; }

    const Node<T>* GetGoalSolutionNode() { return goal_solution_node_; }

    int GetCountIterations() const { return count_iterations_; }

protected:
    const Node<T>* FindBestPath(const T& target_state, Tree<T>& tree_to_search,
                                 int* depth_out) const
    {
        const Node<T>* best_node = nullptr;
        int depth = INT_MAX;

        for (const Node<T>& other : tree_to_search.GetAllNodes())
        {
            double dist =
                start_tree_.GetStateSpace().GetDistance(other.GetState(), target_state);
            if (dist < GetMaxDistanceToGoal() && other.GetDepth() < depth)
            {
                best_node = &other;
                depth = other.GetDepth();
            }
        }

        if (depth_out)
            *depth_out = depth;

        return best_node;
    }

private:
    Tree<T> start_tree_;
    Tree<T> goal_tree_;

    int count_iterations_;
    int min_iterations_;

    int solution_length_;
    const Node<T>* start_solution_node_, *goal_solution_node_;
};

}  // namespace RRT

#endif // JS_TRAJECTORY_PLANNING_NODE_BIRRT_H
