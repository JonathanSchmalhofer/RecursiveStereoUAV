#ifndef JS_TRAJECTORY_PLANNING_NODE_TREE_H
#define JS_TRAJECTORY_PLANNING_NODE_TREE_H

#include <deque>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <functional>
#include <list>
#include <memory>
#include "state_space.h"
#include <stdexcept>
#include <stdlib.h>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace RRT
{
/**
 * Base class for an RRT tree node.
 *
 * @param T The datatype representing the state in the space the RRT
 * will be searching.
 */
template <typename T>
class Node
{
public:
    Node(const T& state, Node<T>* parent = nullptr, int dimensions = 2)
        : parent_(parent), state_(state), vector_(dimensions)
    {
        if (parent_)
        {
            parent_->children_.push_back(this);
        }
        for (int i = 0; i < dimensions; i++)
        {
            vector_[i] = state[i];
        }
        
    }

    const Node<T>* GetParent() const { return parent_; }

    /// @brief Gets the number of ancestors.
    /// 
    /// @detail Gets the number of ancestors (parent, parent's parent, etc) that
    /// the node has.
    /// Returns 0 if it doesn't have a parent.
    int GetDepth() const
    {
        int n = 0;
        for (Node<T>* ancestor = parent_;
             ancestor != nullptr;
             ancestor = ancestor->parent_)
        {
            n++;
        }
        return n;
    }

    /// @brief Get the state of this node.
    /// 
    /// @detail The @state property is the point in the state-space that this
    /// Node represents.  Generally this is a vector (could be 2d, 3d, etc)
    const T& GetState() const { return state_; }

    std::vector<double>* GetCoordinates() { return &vector_; }

private:
    std::vector<double> vector_;
    T state_;
    std::list<Node<T>*> children_;
    Node<T>* parent_;
};

static size_t hash3d(Eigen::Vector3d state); // forward declaration - see 3dspace.h

template <typename T>
class Tree
{
public:
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;
    Tree(std::shared_ptr<StateSpace> state_space, int dimensions)
        : kdtree_(flann::KDTreeSingleIndexParams()),
          dimensions_(dimensions),
          nodemap_(20, RRT::hash3d)
    {
        state_space_ = state_space;
              
        //  default values
        SetStepSize(0.1);
        SetMaxStepSize(5);
        SetMaxIterations(1000);
        SetAdaptiveScalingEnable(false);
        SetGoalBias(0);
        SetWaypointBias(0);
        SetMaxDistanceToGoal(0.1);
    }

    StateSpace& GetStateSpace() { return *state_space_; }
    const StateSpace& GetStateSpace() const { return *state_space_; }

    /// @brief Get the maximum number of iterations.
    /// 
    /// @detail The maximum number of random states in the state-space that we will
    /// try before giving up on finding a path to the goal.
    int GetMaxIterations() const { return max_iterations_; }
    void SetMaxIterations(int iterations) { max_iterations_ = iterations; }

    /// @brief Whether or not the tree is to run with adaptive stepsize control.
    bool IsAdaptiveScalingEnable() const { return adaptive_scaling_enabled_; }
    void SetAdaptiveScalingEnable(bool checked) { adaptive_scaling_enabled_ = checked; }

    /// @brief The chance we extend towards the goal rather than a random point.
    ///
    /// @details At each iteration of the RRT algorithm, we Extend() towards a
    /// particular state.  The goal_bias is a number in the range [0, 1] that
    /// determines what proportion of the time we Extend() towards the goal.
    /// The rest of the time, we Extend() towards a random state.
    double GetGoalBias() const { return goal_bias_; }
    void SetGoalBias(double goal_bias)
    {
        if (goal_bias < 0 || goal_bias > 1)
        {
            throw std::invalid_argument(
                "The goal bias must be a number between 0.0 and 1.0");
        }
        goal_bias_ = goal_bias;
    }

    /// @brief The chance that we extend towards a randomly-chosen point from the
    /// @waypoints vector.
    double GetWaypointBias() const { return waypoint_bias_; }
    void SetWaypointBias(double waypoint_bias)
    {
        if (waypoint_bias < 0 || waypoint_bias > 1)
        {
            throw std::invalid_argument(
                "The waypoint bias must be a number between 0.0 and 1.0");
        }
        waypoint_bias_ = waypoint_bias;
    }

    /// @brief The waypoints vector holds a series of states that were a part of a
    /// previously-generated successful path.
    /// Setting these here and setting @waypoint_bias_ > 0 will bias tree growth
    /// towards these.
    const std::vector<T>& waypoints() const { return waypoints_; }
    void SetWaypoints(const std::vector<T>& waypoints)
    {
        waypoints_ = waypoints;
    }
    void ClearWaypoints() { waypoints_.clear(); }

    double GetStepSize() const { return step_size_; }
    void SetStepSize(double step_size) { step_size_ = step_size; }

    /// @brief Max step size used in ASC
    double GetMaxStepSize() const { return max_step_size_; }
    void SetMaxStepSize(double max_step) { max_step_size_ = max_step; }

    /// @brief How close we have to get to the goal in order to consider it
    /// reached.
    /// @details The RRT will continue to run unti we're within @GetMaxDistanceToGoal of
    /// the goal state reached.
    double GetMaxDistanceToGoal() const { return max_distance_to_goal_; }
    void SetMaxDistanceToGoal(double max_dist) { max_distance_to_goal_ = max_dist; }

    /// @brief Executes the RRT algorithm with the given start state.
    /// @return a bool indicating whether or not it found a path to the goal
    bool Run()
    {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < max_iterations_; i++)
        {
            Node<T>* new_node = Grow();

            if (new_node &&
                state_space_->GetDistance(new_node->GetState(), goal_state_) <
                    max_distance_to_goal_)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    /// @brief Removes nodes from nodes_ and nodemap_ so it can be run() again.
    void Reset(bool erase_root = false)
    {
        kdtree_ = flann::Index<flann::L2_Simple<double>>(
            flann::KDTreeSingleIndexParams());
        if (erase_root)
        {
            nodes_.clear();
            nodemap_.clear();
        }
        else if (nodes_.size() > 1)
        {
            T root = GetRootNode()->GetState();
            nodemap_.clear();
            nodes_.clear();
            nodes_.emplace_back(root, nullptr, dimensions_);
            nodemap_.insert(std::pair<T, Node<T>*>(root, &nodes_.back()));
            kdtree_.buildIndex(flann::Matrix<double>(
                (double*)&(GetRootNode()->GetState()), 1, dimensions_));
        }
    }

    /// @brief Picks a random state and attempts to extend the tree towards it.
    /// This is called at each iteration of the run() method.
    Node<T>* Grow()
    {
        //  extend towards goal, waypoint, or random state depending on the
        //  biases and a random number
        double r =
            rand() /
            (double)RAND_MAX;  //  r is between 0 and one since we normalize it
        if (r < GetGoalBias())
        {
            return Extend(GetGoalState());
        }
        else if (r < GetGoalBias() + GetWaypointBias() && waypoints_.size() > 0)
        {
            const T& waypoint = waypoints_[rand() % waypoints_.size()];
            return Extend(waypoint);
        }
        else
        {
            return Extend(state_space_->GetRandomState());
        }
    }

    /// @brief Find the node int the tree closest to @state.  Pass in a double pointer
    /// as the second argument to get the distance that the node is away from
    /// @state. This method searches a k-d tree of the points to determine
    Node<T>* GetNearest(const T& state, double* distance_out = nullptr)
    {
        Node<T>* best = nullptr;

        // k-NN search (O(log(N)))
        flann::Matrix<double> query;
        query = flann::Matrix<double>((double*)&state, 1,
                                     sizeof(state) / sizeof(0.0));

        std::vector<int> i(query.rows);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(query.rows);
        flann::Matrix<double> dists(d.data(), query.rows, 1);

        int n =
            kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());

        if (distance_out)
            *distance_out = state_space_->GetDistance(state, best->GetState());

        T point;
        point = (T)kdtree_.getPoint(indices[0][0]);

        return nodemap_[point];
    }

    /// @brief Grow the tree in the direction of @state
    /// 
    /// @return the new tree Node (may be nullptr if we hit Obstacles)
    /// @param target The point to extend the tree to
    /// @param source The Node to connect from.  If source == nullptr, then
    /// the closest tree point is used.
    virtual Node<T>* Extend(const T& target, Node<T>* source = nullptr)
    {
        //  if we weren't given a source point, try to find a close node
        if (!source)
        {
            source = GetNearest(target, nullptr);
            if (!source)
            {
                return nullptr;
            }
        }

        //  Get a state that's in the direction of @target from @source. This
        //  should take a step in that direction, but not go all the way unless
        //  the they're really close together.
        T intermediate_state;
        if (adaptive_scaling_enabled_)
        {
            intermediate_state = state_space_->GetIntermediateState(
                source->GetState(), target, GetStepSize(), GetMaxStepSize());
        }
        else
        {
            intermediate_state = state_space_->GetIntermediateState(
                source->GetState(), target, GetStepSize(), GetStepSize());
        }

        //  Make sure there's actually a direct path from @source to
        //  @intermediate_state.  If not, abort
        if (!state_space_->IsTransitionValid(source->GetState(), intermediate_state))
        {
            return nullptr;
        }

        // Add a node to the tree for this state
        nodes_.emplace_back(intermediate_state, source, dimensions_);
        kdtree_.addPoints(flann::Matrix<double>(
            nodes_.back().GetCoordinates()->data(), 1, dimensions_));
        nodemap_.insert(
            std::pair<T, Node<T>*>(intermediate_state, &nodes_.back()));
        return &nodes_.back();
    }

    /// @brief Get the path from the receiver's root point to the dest point
    ///
    ///@param callback The lambda to call for each state in the path
    ///@param dest The node in the tree to get the path for. If nullptr, will
    /// use the the last point added to the @nodes_ vector. If run() was just
    /// called successfully, this node will be the one last created that is
    /// closest to the goal.
    ///@param reverse if true, the states will be sent from @dest to the tree's
    /// root
    void GetPath(std::function<void(const T& stateI)> callback,
                 const Node<T>* dest = nullptr, bool reverse = false) const
    {
        const Node<T>* node = (dest != nullptr) ? dest : GetLastNode();
        if (reverse)
        {
            while (node)
            {
                callback(node->GetState());
                node = node->GetParent();
            }
        }
        else
        {
            // collect states in list in leaf -> root order
            std::vector<const Node<T>*> nodes;
            while (node)
            {
                nodes.push_back(node);
                node = node->GetParent();
            }

            // pass them one-by-one to the callback, reversing the order so
            // that the callback is called with the start point first and the
            // dest point last
            for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++)
            {
                callback((*itr)->GetState());
            }
        }
    }

    /// The same as the first GetPath() method, but appends the states to a given
    /// output vector rather than executing a callback.
    /// 
    /// @param vector_out The vector to append the states along the path
    void GetPath(std::vector<T>* vector_out, const Node<T>* dest = nullptr,
                 bool reverse = false) const
    {
        GetPath([&](const T& stateI) { vector_out->push_back(stateI); }, dest,
                reverse);
    }

    /// The same as the first GetPath() method, but returns the vector of states
    /// instead of executing a callback.
    std::vector<T> GetPath(const Node<T>* dest = nullptr,
                           bool reverse = false) const
    {
        std::vector<T> path;
        GetPath(&path, dest, reverse);
        return path;
    }

    /// @brief Get the root node.
    /// 
    /// @return The root node or nullptr if none exists
    const Node<T>* GetRootNode() const {
        if (nodes_.empty()) return nullptr;

        return &nodes_.front();
    }

    /// @brief Get the most recent Node added to the tree.
    const Node<T>* GetLastNode() const
    {
        if (nodes_.empty()) return nullptr;
            return &nodes_.back();
    }

    /// @brief Get all the nodes
    const std::deque<Node<T>>& GetAllNodes() const
    {
        return nodes_;
    }

    /// @brief The start state for this tree
    const T& GetStartState() const
    {
        if (nodes_.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return GetRootNode()->GetState();
    }
    void SetStartState(const T& start_state)
    {
        Reset(true);

        //  create root node from provided start state
        nodes_.emplace_back(start_state, nullptr, dimensions_);
        nodemap_.insert(std::pair<T, Node<T>*>(start_state, &nodes_.back()));
        kdtree_.buildIndex(flann::Matrix<double>(
            (double*)&(GetRootNode()->GetState()), 1, dimensions_));
        
    }

    /// @brief The goal this tree is trying to reach.
    const T& GetGoalState() const { return goal_state_; }
    void SetGoalState(const T& goal_state) { goal_state_ = goal_state; }

protected:
    /// @brief A list of all Node objects in the tree.
    std::deque<Node<T>> nodes_{};

    std::unordered_map<T, Node<T>*, std::function<size_t(T)>> nodemap_;

    T goal_state_;

    const int dimensions_;

    int max_iterations_;

    bool adaptive_scaling_enabled_;

    double goal_bias_;

    /// used for Extended RRTs where growth is biased towards waypoints from
    /// previously grown tree
    double waypoint_bias_;
    std::vector<T> waypoints_{};

    double max_distance_to_goal_;

    double step_size_;
    double max_step_size_;

    flann::Index<flann::L2_Simple<double>> kdtree_;

    std::shared_ptr<StateSpace> state_space_{};
};
}  // namespace RRT

#endif // JS_TRAJECTORY_PLANNING_NODE_TREE_H
