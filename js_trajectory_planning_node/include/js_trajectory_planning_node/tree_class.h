///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief A tree representation based on https://github.com/RoboJackets/rrt/blob/master/src/rrt/Tree.hpp
///
#ifndef JS_TRAJECTORY_PLANNING_NODE_TREE_CLASS_H_
#define JS_TRAJECTORY_PLANNING_NODE_TREE_CLASS_H_

#include "planning_space_3d_class.h"    // PlanningSpace3d

namespace js_trajectory_planning_node
{

class Tree {
public:
    Tree(const Tree&) = delete;
    
    Tree& operator=(const Tree&) = delete;
    
    Tree(std::shared_ptr<PlanningSpace3d> planning_space)
    {
        planning_space_ = planning_space;
    }

    PlanningSpace3d& GetPlanningSpace()
    {
        return *planning_space_;
    }
    const PlanningSpace3d& GetPlanningSpace() const
    {
        return *planning_space_;
    }

    /**
     * Executes the RRT algorithm with the given start state.
     *
     * @return a bool indicating whether or not it found a path to the goal
     */
    bool run() {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < _maxIterations; i++) {
            Node<T>* newNode = grow();

            if (newNode &&
                _stateSpace->distance(newNode->state(), _goalState) <
                    _goalMaxDist)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    /**
     * Removes nodes from nodes_ so it can be run() again.
     */
    void reset()
    {
        nodes_.clear();
    }

    /**
     * Find the node int the tree closest to @state.  Pass in a double pointer
     * as the second argument to get the distance that the node is away from
     * @x_in. This method searches a k-d tree of the points to determine
     */
    Node3d* nearest(const Node3d& x_in, double& tree_node_distance)
    {
        Node3d* x_closest = nullptr;
        
        double min_distance = std::numeric_limits<double>::infinity();
        for(auto& tree_node : nodes_)
        {
            if(tree_node != x_in)
            {
                tree_node_distance = EuclidianDistance3d(tree_node, x_in);
                if (tree_node_distance < min_distance) {
                    min_distance = tree_node_distance;
                    closest_node = tree_node;
                }
            }
        }
        tree_node_distance = 

        return _nodemap[point];
    }

    /**
     * Grow the tree in the direction of @state
     *
     * @return the new tree Node (may be nullptr if we hit Obstacles)
     * @param target The point to extend the tree to
     * @param source The Node to connect from.  If source == nullptr, then
     *             the closest tree point is used
     */
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
        //  if we weren't given a source point, try to find a close node
        if (!source) {
            source = nearest(target, nullptr);
            if (!source) {
                return nullptr;
            }
        }

        //  Get a state that's in the direction of @target from @source. This
        //  should take a step in that direction, but not go all the way unless
        //  the they're really close together.
        T intermediateState;
        if (_isASCEnabled) {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize(), maxStepSize());
        } else {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize());
        }

        //  Make sure there's actually a direct path from @source to
        //  @intermediateState.  If not, abort
        if (!_stateSpace->transitionValid(source->state(), intermediateState)) {
            return nullptr;
        }

        // Add a node to the tree for this state
        _nodes.emplace_back(intermediateState, source, _dimensions, _TToArray);
        _kdtree.addPoints(flann::Matrix<double>(
            _nodes.back().coordinates()->data(), 1, _dimensions));
        _nodemap.insert(
            std::pair<T, Node<T>*>(intermediateState, &_nodes.back()));
        return &_nodes.back();
    }

    /**
     * Get the path from the receiver's root point to the dest point
     *
     * @param callback The lambda to call for each state in the path
     * @param dest The node in the tree to get the path for. If nullptr, will
     *     use the the last point added to the @_nodes vector. If run() was just
     *     called successfully, this node will be the one last created that is
     *     closest to the goal.
     * @param reverse if true, the states will be sent from @dest to the tree's
     *     root
     */
    void getPath(std::function<void(const T& stateI)> callback,
                 const Node<T>* dest = nullptr, bool reverse = false) const {
        const Node<T>* node = (dest != nullptr) ? dest : lastNode();
        if (reverse) {
            while (node) {
                callback(node->state());
                node = node->parent();
            }
        } else {
            // collect states in list in leaf -> root order
            std::vector<const Node<T>*> nodes;
            while (node) {
                nodes.push_back(node);
                node = node->parent();
            }

            // pass them one-by-one to the callback, reversing the order so
            // that the callback is called with the start point first and the
            // dest point last
            for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++) {
                callback((*itr)->state());
            }
        }
    }

    /**
     * The same as the first getPath() method, but appends the states to a given
     * output vector rather than executing a callback.
     *
     * @param vectorOut The vector to append the states along the path
     */
    void getPath(std::vector<T>* vectorOut, const Node<T>* dest = nullptr,
                 bool reverse = false) const {
        getPath([&](const T& stateI) { vectorOut->push_back(stateI); }, dest,
                reverse);
    }

    /**
     * The same as the first getPath() method, but returns the vector of states
     * instead of executing a callback.
     */
    std::vector<T> getPath(const Node<T>* dest = nullptr,
                           bool reverse = false) const {
        std::vector<T> path;
        getPath(&path, dest, reverse);
        return path;
    }

    /**
     * @return The root node or nullptr if none exists
     */
    const Node<T>* rootNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.front();
    }

    /**
     * @return The most recent Node added to the tree
     */
    const Node<T>* lastNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.back();
    }

    /**
     * All the nodes
     */
    const std::deque<Node<T>>& allNodes() const { return _nodes; }

    /**
     * @brief The start state for this tree
     */
    const T& startState() const {
        if (_nodes.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return rootNode()->state();
    }
    void setStartState(const T& startState) {
        reset(true);

        //  create root node from provided start state
        _nodes.emplace_back(startState, nullptr, _dimensions, _TToArray);
        _nodemap.insert(std::pair<T, Node<T>*>(startState, &_nodes.back()));
        if (_TToArray) {
            std::vector<double> data(_dimensions);
            _TToArray(rootNode()->state(), data.data());
            _kdtree.buildIndex(
                flann::Matrix<double>(data.data(), 1, _dimensions));
        } else {
            _kdtree.buildIndex(flann::Matrix<double>(
                (double*)&(rootNode()->state()), 1, _dimensions));
        }
    }

    /**
     * @brief The goal this tree is trying to reach.
     */
    const T& goalState() const { return _goalState; }
    void setGoalState(const T& goalState) { _goalState = goalState; }

protected:
    /**
     * A list of all Node objects in the tree.
     */
    std::deque<Node3d> nodes_{};

    std::unordered_map<T, Node<T>*, std::function<size_t(T)>> _nodemap;

    T _goalState;

    const int _dimensions;

    int _maxIterations;

    bool _isASCEnabled;

    double _goalBias;

    /// used for Extended RRTs where growth is biased towards waypoints from
    /// previously grown tree
    double _waypointBias;
    std::vector<T> _waypoints{};

    double _goalMaxDist;

    double _stepSize;
    double _maxStepSize;

    flann::Index<flann::L2_Simple<double>> _kdtree;

    std::function<T(double*)> _arrayToT;

    std::function<void(T, double*)> _TToArray;

    std::shared_ptr<StateSpace<T>> _stateSpace{};
};

}  // namespace js_trajectory_planning_node

#endif  // JS_TRAJECTORY_PLANNING_NODE_TREE_CLASS_H_
