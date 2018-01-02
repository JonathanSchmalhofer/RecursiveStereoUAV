///
/// @file
/// @copyright Copyright (C) 2017, Jonathan Bryan Schmalhofer
///
/// @brief RT-RRT* class based on https://github.com/rishabh1b/RealTimePathPlanning/
///
#include "js_trajectory_planning_node/rtrrtstar_class.h"

namespace js_trajectory_planning_node
{

RTRRTStarClass::RTRRTStarClass()
{
    Initialize();
    FinalizeLoopCycle();
}

RTRRTStarClass::~RTRRTStarClass()
{
    Initialize();
}

void RTRRTStarClass::Initialize()
{
    T.clear();
    Q_s.clear();
    Q_r.clear();
    Xi_obs.node_space_.clear();
    UpdateXAgent(T.insert(T.begin(), NodeData(Vector3d(kminimum_uniform_extent_x, 0, 0))));
    UpdateXGoal(T.insert(T.begin(), NodeData(Vector3d(0, kminimum_uniform_extent_y, 0))));
    UpdateX0(T.insert(T.begin(), NodeData(Vector3d(0, 0, 0), 0)));
}

void RTRRTStarClass::PerformPlanningCycleOnce()
{
    ROS_INFO("====================================================");
    ROS_INFO("Current tree:");
    IndentedRosInfo(T);
    ROS_INFO("----------------------------------------------------");
    
    //x_goal = Node(Vector3d(1,10,100)); //Todo: remove this line, just for testing
    while(IsTimeLeftForExpansionAndRewiring())
    {
        ROS_INFO("Expand and Rewire");
        ExpandAndRewireTree();
    }
    //ROS_INFO("And Now?");
    PlanPathForKSteps();
    if(IsAgentCloseToTreeRoot())
    {
        ChangeTreeRootToNextImmediateNode();
    }
    FinalizeLoopCycle();
}

void RTRRTStarClass::UpdateXAgent(NodeIt x_in)
{
    x_agent = x_in;
}

void RTRRTStarClass::UpdateXGoal(NodeIt x_in)
{
    x_goal = x_in;
}

void RTRRTStarClass::UpdateX0(NodeIt x_in)
{
    x_0 = x_in;
}

void RTRRTStarClass::UpdateXiObs()
{
}

void RTRRTStarClass::UpdateXiFree()
{
}

std::string RTRRTStarClass::ToString(const NodeIt& x_in)
{
    return std::string("(x,y,z) = (" +
                       std::to_string((*x_in).position_.x_) + "," +
                       std::to_string((*x_in).position_.y_) + "," +
                       std::to_string((*x_in).position_.z_) + ") cost_to_start_ = " + std::to_string((*x_in).cost_to_start_));
}

void RTRRTStarClass::IndentedRosInfo(const tree<NodeData>& tree_in)
{
    tree<NodeData>::pre_order_iterator it = tree_in.begin();
    tree<NodeData>::pre_order_iterator end = tree_in.end();
    if(!tree_in.is_valid(it))
    {
        return;
    }

    int rootdepth = tree_in.depth(it);

    while(it!=end)
    {
        std::string node_info_string;
        for(int i=0; i < tree_in.depth(it)-rootdepth; ++i)
        {
            node_info_string.append("  ");
        }
        ROS_INFO(node_info_string.append(ToString(it)).c_str());
        ++it;
    }
}

void RTRRTStarClass::NodeListRosInfo(const std::list<NodeIt>& list_in)
{
    ROS_INFO("List:");
    for (auto& node_from_list : list_in)
    {
        ROS_INFO(std::string("   " + ToString(node_from_list)).c_str());
    }
}

void RTRRTStarClass::ExpandAndRewireTree()
{
    NodeIt x_rand = SampleRandom();
    NodeIt x_closest = GetClosestNodeInTree(x_rand);
    
    if(CheckIfCollisionFreeLineBetween(x_closest, x_rand))
    {
        std::list<NodeIt> Xi_near = FindNodesNear3d(x_rand);
        if(     Xi_near.size() < kmaximum_number_closest_neighbours
          ||    EuclidianDistance3d(x_closest, x_rand) > kradius_closest_neighbours)
        {
            x_rand = AddNodeToTree(x_rand, x_closest, Xi_near);
            Q_r.push_front(x_rand);
        }
        else
        {
            Q_r.push_front(x_closest);
        }
        ROS_INFO("RewireRandomNodes");
        RewireRandomNodes();
        //ROS_INFO("Q_r.size() = %zd", Q_r.size());//Todo: delete
    }
    ROS_INFO("RewireFromTreeRoot");
    RewireFromTreeRoot();
    //ROS_INFO("Q_s.size() = %zd", Q_s.size());//Todo: delete
    ++counter_expansions_and_rewiring_;
}

void RTRRTStarClass::RewireRandomNodes()
{
    while(IsTimeLeftForRewireRandomNodes() && !Q_r.empty() ) // Todo: add another iterator limit (e.g. check for time)
    {
        //ROS_INFO("Keep rewiring");
        NodeIt x_r = Q_r.front();
        Q_r.pop_front();
        std::list<NodeIt> Xi_near = FindNodesNear3d(x_r);
        double c_old = std::numeric_limits<double>::infinity();
        double c_new = std::numeric_limits<double>::infinity();
        for (auto& x_near : Xi_near)
        {
            c_old = cost(x_near);
            c_new = cost(x_r) + EuclidianDistance3d(x_r, x_near);
            if(     c_new < c_old
               &&   CheckIfCollisionFreeLineBetween(x_near, x_r))
            {
                // add edge from x_r to x_near and remove edge Parent(x_near) to x_near 
                x_near = ChangeParent(x_near, x_r);
                // update costs
                (*x_near).cost_to_start_ = c_new;
                
                Q_r.push_back(x_near);
            }
        }
        ++counter_rewire_random_nodes_;
    }
}

void RTRRTStarClass::RewireFromTreeRoot()
{
    if(Q_s.empty())
    {
        Q_s.push_back(x_0);
    }
    while(IsTimeLeftForRewireFromTreeRoot() && !Q_s.empty() ) // Todo: add another iterator limit (e.g. check for time)
    {
        ++counter_rewire_from_tree_root_;
    }
}

std::list<NodeIt> RTRRTStarClass::FindNodesNear3d(const NodeIt& x_in)
{
    std::list<NodeIt> Xi_near;
    
    // Calculate epsilon for 3d
    const double pi = std::acos(-1);
    double numerator = 3.0f * GetVolumeOfSearchSpace3d() * kmaximum_number_closest_neighbours;
    double denominator = 4.0f * pi * T.size();
    double epsilon = std::pow(numerator/denominator, 1/3);
    
    if(epsilon < kradius_closest_neighbours)
    {
        epsilon = kradius_closest_neighbours;
    }

    for(NodeIt tree_node = T.begin(); tree_node != T.end(); tree_node++)
    {
        if(Xi_near.size() >= kmaximum_number_closest_neighbours)
        {
            break;
        }
        if(EuclidianDistance3d(tree_node, x_in) < epsilon)
        {
            if(     tree_node != x_in
               &&   tree_node != x_agent
               &&   tree_node != x_goal)
            {
                Xi_near.push_back(tree_node);
            }
        }
    }
    return Xi_near;
}

double RTRRTStarClass::cost(const NodeIt& x_in)
{
    bool blocked_node = false;
    float cumulative_cost = 0;
    NodeIt current_node = x_in;
    NodeIt parent_node = static_cast<NodeIt>(T.parent(current_node));
    while(parent_node != NULL)
    {
        if((*parent_node).cost_to_start_ >= std::numeric_limits<double>::infinity())
        {
            blocked_node = true;
            break;
        }
        cumulative_cost += EuclidianDistance3d(current_node, parent_node);
        current_node = parent_node;
        parent_node = T.parent(current_node);
    }
    if (blocked_node)
    {
        return std::numeric_limits<double>::infinity();
    }
    else
    {
        return cumulative_cost;
    }
}

double RTRRTStarClass::GetVolumeOfSearchSpace3d()
{
    octomap::point3d max_point;
    octomap::point3d min_point;
    
    if(Xi_obs.octomap_space_ != NULL)
    {
        max_point = Xi_obs.octomap_space_->getBBXMax();
        min_point = Xi_obs.octomap_space_->getBBXMin();
    }
    
    double x_upper_bound = std::max(+0.5*kminimum_uniform_extent_x,static_cast<double>(max_point.x()));
    double x_lower_bound = std::min(-0.5*kminimum_uniform_extent_x,static_cast<double>(min_point.x()));
    
    double y_upper_bound = std::max(+0.5*kminimum_uniform_extent_y,static_cast<double>(max_point.y()));
    double y_lower_bound = std::min(-0.5*kminimum_uniform_extent_y,static_cast<double>(min_point.y()));
    
    double z_upper_bound = std::max(+0.5*kminimum_uniform_extent_z,static_cast<double>(max_point.z()));
    double z_lower_bound = std::min(-0.5*kminimum_uniform_extent_z,static_cast<double>(min_point.z()));
    
    double x_side = x_upper_bound - x_lower_bound;
    double y_side = y_upper_bound - y_lower_bound;
    double z_side = z_upper_bound - z_lower_bound;
    
    return (x_side * y_side * z_side);
}

NodeIt RTRRTStarClass::AddNodeToTree(NodeIt& x_new, const NodeIt& x_closest, std::list<NodeIt> Xi_near)
{
    //ROS_INFO("AddNodeToTree");
    NodeIt x_min = x_closest;
    double c_min = cost(x_closest) + EuclidianDistance3d(x_new, x_closest);
    double c_new = 0;
    for (auto& x_near : Xi_near)
    {
        c_new = cost(x_near) + EuclidianDistance3d(x_new, x_near);
        if(     c_new < c_min
           &&   CheckIfCollisionFreeLineBetween(x_near, x_new))
        {
            c_min = c_new;
            x_min = x_near;
        }
    }
    // adding the node to tree T is not necessary, as it already is unconnected node in T
    // so we only need to add the connections/append the node to the correct parent node

    // add edge from from x_new to x_min...
    (*x_new).cost_to_start_ = c_min;
    x_new = ChangeParent(x_new, x_min);
    
    return x_new;
}

void RTRRTStarClass::PlanPathForKSteps()
{
}

bool RTRRTStarClass::IsTimeLeftForExpansionAndRewiring()
{
    bool is_time_left = false;
    if (counter_expansions_and_rewiring_ < kmax_number_expansions_and_rewiring)
    {
        is_time_left = true;
    }
    
    // Todo: add another iterator limit (e.g. check for time)
    
    return is_time_left;
}

bool RTRRTStarClass::IsTimeLeftForRewireRandomNodes()
{
    bool is_time_left = false;
    if (counter_rewire_random_nodes_ < kmax_number_rewire_random_nodes)
    {
        is_time_left = true;
    }
    
    // Todo: add another iterator limit (e.g. check for time)
    
    return is_time_left;
}

bool RTRRTStarClass::IsTimeLeftForRewireFromTreeRoot()
{
    bool is_time_left = false;
    if (counter_rewire_from_tree_root_ < kmax_number_rewire_from_tree_root)
    {
        is_time_left = true;
    }
    
    // Todo: add another iterator limit (e.g. check for time)
    
    return is_time_left;
}

bool RTRRTStarClass::IsAgentCloseToTreeRoot()
{
    bool is_close = false;
    
    return is_close;
}

void RTRRTStarClass::ChangeTreeRootToNextImmediateNode()
{
}

void RTRRTStarClass::FinalizeLoopCycle()
{
    counter_expansions_and_rewiring_ = 0;
    counter_rewire_random_nodes_ = 0;
    counter_rewire_from_tree_root_ = 0;
}

NodeIt RTRRTStarClass::SampleRandom()
{
    double p_r = UniformRandomNumberBetween(0.0f, 1.0f);
    
    if (p_r > 1-kalpha)
	{
        if(EuclidianDistance3d(x_0, x_goal) > 0)
        {
            //ROS_INFO("LineTo"); //Todo: delete
            return LineTo(x_goal);
        }
        else
        {
            //ROS_INFO("Uniform instead of LineTo"); //Todo: delete
            return Uniform();
        }
        
	}
	else if (p_r <= ((1-kalpha)/kbeta))
	{
        //ROS_INFO("Uniform"); //Todo: delete
        return Uniform();
	}
	else
	{
        
        if(EuclidianDistance3d(x_0, x_goal) > 0)
        {
            //ROS_INFO("Ellipsoid"); //Todo: delete
            return Ellipsoid(x_0, x_goal);
        }
        else
        {
            //ROS_INFO("Uniform instead of Ellipsoid"); //Todo: delete
            return Uniform();
        }
	}
}

    NodeIt RTRRTStarClass::LineTo(const NodeIt& x_in)
{
    NodeIt closest_node_in_tree = GetClosestNodeInTree(x_in);
    double scale_factor = UniformRandomNumberBetween(0.0f, 1.0f);
    
    // Scale between x_in and closest_node_in_tree
    // see: https://math.stackexchange.com/questions/2045174/how-to-find-a-point-between-two-points-with-given-distance
    double x_new = (*x_in).position_.x_ + scale_factor * ((*closest_node_in_tree).position_.x_-(*x_in).position_.x_);
    double y_new = (*x_in).position_.y_ + scale_factor * ((*closest_node_in_tree).position_.y_-(*x_in).position_.y_);
    double z_new = (*x_in).position_.z_ + scale_factor * ((*closest_node_in_tree).position_.z_-(*x_in).position_.z_);
    
    return T.insert(T.begin(),NodeData(Vector3d(x_new, y_new, z_new)));
}

NodeIt RTRRTStarClass::Uniform()
{
    octomap::point3d max_point;
    octomap::point3d min_point;
    
    if(Xi_obs.octomap_space_ != NULL)
    {
        max_point = Xi_obs.octomap_space_->getBBXMax();
        min_point = Xi_obs.octomap_space_->getBBXMin();
    }
    
    double x_upper_bound = std::max(+0.5*kminimum_uniform_extent_x,static_cast<double>(max_point.x()));
    double x_lower_bound = std::min(-0.5*kminimum_uniform_extent_x,static_cast<double>(min_point.x()));
    double x_new = UniformRandomNumberBetween(x_lower_bound, x_upper_bound);
    
    double y_upper_bound = std::max(+0.5*kminimum_uniform_extent_y,static_cast<double>(max_point.y()));
    double y_lower_bound = std::min(-0.5*kminimum_uniform_extent_y,static_cast<double>(min_point.y()));
    double y_new = UniformRandomNumberBetween(y_lower_bound, y_upper_bound);
    
    double z_upper_bound = std::max(+0.5*kminimum_uniform_extent_z,static_cast<double>(max_point.z()));
    double z_lower_bound = std::min(-0.5*kminimum_uniform_extent_z,static_cast<double>(min_point.z()));
    double z_new = UniformRandomNumberBetween(z_lower_bound, z_upper_bound);


    return T.insert(T.begin(), NodeData(Vector3d(x_new, y_new, z_new)));
}

NodeIt RTRRTStarClass::Ellipsoid(const NodeIt& x_a, const NodeIt& x_b)
{
    Eigen::Vector3d x_a_eigen = Eigen::Vector3d((*x_a).position_.x_, (*x_a).position_.y_, (*x_a).position_.z_);
    Eigen::Vector3d x_b_eigen = Eigen::Vector3d((*x_b).position_.x_, (*x_b).position_.y_, (*x_b).position_.z_);
    Eigen::Vector3d x_center = 0.5 * (x_a_eigen + x_b_eigen);
    Eigen::Vector3d direction = x_b_eigen - x_a_eigen;
    
    std::vector<double> abs_direction = {std::abs(direction[0]), std::abs(direction[1]), std::abs(direction[2])};
    std::sort(abs_direction.begin(), abs_direction.end());
    
    // Calculation of covariance, compare: http://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/geometry/simple.html    
    double a = abs_direction[2];
    double b = abs_direction[1];
    double c = abs_direction[0];
    
    // Todo: consider using c_best and c_min as explained in paper from Naderi et al
    
    Eigen::Matrix< double, 3, 1> v;
    v << a, a, b; // only use two largest directions
    Eigen::Matrix3d covariance = v.array().matrix().asDiagonal();

    return T.insert(T.begin(),NodeData(DrawWithinEllipsoid(covariance, x_center)));
}

Vector3d RTRRTStarClass::DrawWithinEllipsoid(const Eigen::Matrix3d covariance, const Eigen::Vector3d center)
{
    // Algorithm see: http://www.astro.gla.ac.uk/~matthew/blog/?p=368
    
    // calculate eigenvalues and vectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.compute(covariance);
    
    // generate radius of hypersphere
    double radius_hypersphere = UniformRandomNumberBetween(0.0f, 1.0f);
    
    // generate point
    Eigen::Vector3d point(NormalRandomNumber(), NormalRandomNumber(), NormalRandomNumber());
    
    // get scaling for point onto the surface of a unit hypersphere
    double factor_scaling = std::pow(point[0],2) + std::pow(point[1],2) + std::pow(point[2],2);
    factor_scaling = std::pow(radius_hypersphere,1/3) / std::sqrt(factor_scaling);
    // scale point to the ellipsoid using the eigenvalues and rotate with the eigenvectors and add centroid
    Eigen::Vector3d sqrt_d = solver.eigenvalues().cwiseSqrt();
    Eigen::Vector3d drawn_point = factor_scaling * point;
    
    // scale and rotate to ellipsoid
    Eigen::Matrix3d v = solver.eigenvectors();    
    Eigen::Vector3d rotation = sqrt_d.transpose() * v.transpose();
    Eigen::Vector3d result = point.cwiseProduct(rotation) + center;

    return Vector3d(result[0], result[1], result[2]);
}

NodeIt RTRRTStarClass::GetClosestNodeInTree(const NodeIt& x_in)
{
    //ROS_INFO("GetClosestNodeInTree()");
    NodeIt closest_node = x_in;
    double min_distance = std::numeric_limits<double>::infinity();
    double tree_node_distance;
    for(NodeIt tree_node = T.begin(); tree_node != T.end(); tree_node++)
    {
        if(     tree_node != x_in
           &&   tree_node != x_agent
           &&   tree_node != x_goal)
        {
            //ROS_INFO(std::string(" Checking " + ToString(tree_node)).c_str());
            tree_node_distance = EuclidianDistance3d(tree_node, x_in);
            if (tree_node_distance < min_distance) {
                min_distance = tree_node_distance;
                closest_node = tree_node;
            }
        }
    }
    
    return closest_node;
}

bool RTRRTStarClass::CheckIfCollisionFreeLineBetween(const NodeIt& x_a, const NodeIt& x_b)
{
    if(Xi_obs.octomap_space_ == NULL)
    {
        return true;
    }
    else
    {
        octomap::point3d start((*x_a).position_.x_,
                               (*x_a).position_.y_,
                               (*x_a).position_.z_);
        octomap::point3d direction((*x_b).position_.x_ - (*x_a).position_.x_,
                                   (*x_b).position_.y_ - (*x_a).position_.y_,
                                   (*x_b).position_.z_ - (*x_a).position_.z_);
        octomap::point3d cell_hit_by_ray;
        
        ROS_INFO("Casting ray");
        return Xi_obs.octomap_space_->castRay(start, direction, cell_hit_by_ray);
    }
}

double RTRRTStarClass::EuclidianDistance3d(const NodeIt& a, const NodeIt& b)
{
    return std::sqrt( std::pow(((*a).position_.x_-(*b).position_.x_), 2) + std::pow(((*a).position_.y_-(*b).position_.y_), 2) + std::pow(((*a).position_.z_-(*b).position_.z_), 2));;
}

double RTRRTStarClass::UniformRandomNumberBetween(const double a, const double b)
{
    std::random_device temp_random_device;              // Will be used to obtain a seed for the random number engine
    std::mt19937 temp_generator(temp_random_device());  // Standard mersenne_twister_engine seeded with temp_random_device()
    std::uniform_real_distribution<double> temp_distribution(a, b);
    return temp_distribution(temp_generator);
}

double RTRRTStarClass::NormalRandomNumber()
{
    std::random_device temp_random_device;              // Will be used to obtain a seed for the random number engine
    std::mt19937 temp_generator(temp_random_device());  // Standard mersenne_twister_engine seeded with temp_random_device()
    std::normal_distribution<double> temp_distribution(0.0f, 1.0f);
    return temp_distribution(temp_generator);
}

NodeIt RTRRTStarClass::ChangeParent(NodeIt& child_node, const NodeIt& new_parent)
{
    /*
    tree<NodeData> child_subtree = T.move_out(child_node);
    ROS_INFO("ChangeParent - Check 1");
    child_subtree.debug_verify_consistency();
    ROS_INFO("ChangeParent - Check 2");
    T.debug_verify_consistency();
    //Todo: Decide whether to append as first or last child?
    // child_node = T.move_in_as_nth_child(new_parent, T.number_of_children(new_parent), child_subtree);
    child_node = T.move_in_as_nth_child(new_parent, 0, child_subtree);
    ROS_INFO("ChangeParent - Check 3");
    T.debug_verify_consistency();	
    return child_node;
    */
    ROS_INFO("ChangeParent - Check 1");
    T.debug_verify_consistency();
    auto new_child = T.append_child(new_parent);
    ROS_INFO("ChangeParent - Check 2");
    T.debug_verify_consistency();
    child_node = T.move_ontop(new_child, child_node);
    ROS_INFO("ChangeParent - Check 3");
    T.debug_verify_consistency();
    return child_node;
}



}  // namespace js_trajectory_planning_node
