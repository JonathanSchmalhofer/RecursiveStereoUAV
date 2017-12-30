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
    counter_expansions_and_rewiring_ = 0;
}

RTRRTStarClass::~RTRRTStarClass()
{
}

void RTRRTStarClass::PerformPlanningCycleOnce()
{
    x_goal = Node(Vector3d(1,10,100));
    while(IsTimeLeftForExpansionAndRewiring())
    {
        ExpandAndRewireTree();
    }
    PlanPathForKSteps();
    if(IsAgentCloseToTreeRoot())
    {
        ChangeTreeRootToNextImmediateNode();
    }
    FinalizeLoopCycle();
}

void RTRRTStarClass::UpdateXAgent(Node x_agent)
{
}

void RTRRTStarClass::UpdateXGoal(Node x_goal)
{
}

void RTRRTStarClass::UpdateXiObs()
{
}

void RTRRTStarClass::UpdateXiFree()
{
}

void RTRRTStarClass::ExpandAndRewireTree()
{
    Node x_rand = SampleRandom();
    Node& x_closest = GetClosestNodeInTree(x_rand);
    if(CheckIfCollisionFreeLineBetween(x_closest, x_rand))
    {
        std::list<std::reference_wrapper<Node>> Xi_near = FindNodesNear3d(x_rand);
        if(     Xi_near.size() < kmaximum_number_closest_neighbours
          ||    EuclidianDistance3d(x_closest, x_rand) > kradius_closest_neighbours)
        {
            AddNodeToTree(x_rand, x_closest, Xi_near);
            //Todo: delete//ROS_INFO("AddNodeToTree");
            //Todo: delete//ROS_INFO("T.size() = %zd", T.size());
            Q_r.push_front(std::ref(x_rand));
        }
        else
        {
            Q_r.push_front(std::ref(x_closest));
        }
        RewireRandomNodes();
    }
    RewireFromTreeRoot();
    ++counter_expansions_and_rewiring_;
}

void RTRRTStarClass::RewireRandomNodes()
{
    while(!Q_r.empty()) // Todo: add another iterator limit
    {
        Node& x_r = Q_r.front().get();
        Q_r.pop_front();
        std::list<std::reference_wrapper<Node>> Xi_near = FindNodesNear3d(x_r);
        double c_old = std::numeric_limits<double>::infinity();
        double c_new = std::numeric_limits<double>::infinity();
        for (auto& ref_x_near : Xi_near)
        {
            c_old = cost(ref_x_near.get());
            c_new = cost(x_r) + EuclidianDistance3d(x_r, ref_x_near.get());
            if(     c_new < c_old
               &&   CheckIfCollisionFreeLineBetween(ref_x_near.get(), x_r))
            {
                /*
                (*it)->prevParent = (*it)->parent;
				(*it)->parent->children.remove(*it);
				(*it)->parent = Xr;
				(*it)->costToStart = newCost;
				Xr->children.push_back(*it);
                */
                Q_r.push_back(std::ref(ref_x_near.get()));
            }
        }
    }
}

void RTRRTStarClass::RewireFromTreeRoot()
{
}

std::list<std::reference_wrapper<Node>> RTRRTStarClass::FindNodesNear3d(Node x_in)
{
    std::list<std::reference_wrapper<Node>> Xi_near;
    
    // Calculate epsilon for 3d
    const double pi = std::acos(-1);
    double numerator = 3.0f * GetVolumeOfSearchSpace3d() * kmaximum_number_closest_neighbours;
    double denominator = 4.0f * pi * T.size();
    double epsilon = std::pow(numerator/denominator, 1/3);
    
    if(epsilon < kradius_closest_neighbours)
    {
        epsilon = kradius_closest_neighbours;
    }
    
    for (auto& tree_node : T)
    {
        if(Xi_near.size() >= kmaximum_number_closest_neighbours)
        {
            break;
        }
        if(EuclidianDistance3d(tree_node, x_in) < epsilon)
        {
            Xi_near.push_back(std::ref(tree_node));
        }
    }
    return Xi_near;
}

double RTRRTStarClass::cost(Node& x_in)
{
    bool blocked_node = false;
    float cumulative_cost = 0;
    Node& current_node = x_in;
    while(current_node.parent_ != NULL)
    {
        if(current_node.parent_->cost_to_start_ >= std::numeric_limits<double>::infinity())
        {
            x_in.cost_to_start_ = std::numeric_limits<double>::infinity();
            blocked_node = true;
            break;
        }
        cumulative_cost += EuclidianDistance3d(current_node, *(current_node.parent_));
        current_node = *(current_node.parent_);
    }
    if (blocked_node)
    {
        return std::numeric_limits<double>::infinity();
    }
    else
    {
        x_in.cost_to_start_ = cumulative_cost;
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

void RTRRTStarClass::AddNodeToTree(Node& x_new, Node& x_closest, std::list<std::reference_wrapper<Node>> Xi_near)
{
    Node& x_min = x_closest;
    double c_min = cost(x_closest) + EuclidianDistance3d(x_new, x_closest);
    double c_new = 0;
    for (auto& ref_x_near : Xi_near)
    {
        c_new = cost(ref_x_near.get()) + EuclidianDistance3d(x_new, ref_x_near.get());
        if(     c_new < c_min
           &&   CheckIfCollisionFreeLineBetween(ref_x_near.get(), x_new))
        {
            c_min = c_new;
            x_min = ref_x_near.get();
        }
    }
    x_new.cost_to_start_ = c_min;
    T.push_back(x_new);
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
}

Node RTRRTStarClass::SampleRandom()
{
    double p_r = UniformRandomNumberBetween(0.0f, 1.0f);
    
    if (p_r > 1-kalpha)
	{
        //Todo: delete //ROS_INFO("LineTo");
        return LineTo(x_goal);
	}
	else if (p_r <= ((1-kalpha)/kbeta))
	{
        //Todo: delete //ROS_INFO("Uniform");
        return Uniform();
	}
	else
	{
        
        if(EuclidianDistance3d(x_0, x_goal) > 0)
        {
            //Todo: delete //ROS_INFO("Ellipsoid");
            return Ellipsoid(x_0, x_goal);
        }
        else
        {
            //Todo: delete //ROS_INFO("Uniform instead of Ellipsoid");
            return Uniform();
        }
	}
    
}

Node RTRRTStarClass::LineTo(Node x_in)
{
    Node& closest_node_in_tree = GetClosestNodeInTree(x_in);    
    double scale_factor = UniformRandomNumberBetween(0.0f, 1.0f);
    
    // Scale between x_in and closest_node_in_tree
    // see: https://math.stackexchange.com/questions/2045174/how-to-find-a-point-between-two-points-with-given-distance
    double x_new = x_in.position_.x_ + scale_factor * (closest_node_in_tree.position_.x_-x_in.position_.x_);
    double y_new = x_in.position_.y_ + scale_factor * (closest_node_in_tree.position_.y_-x_in.position_.y_);
    double z_new = x_in.position_.z_ + scale_factor * (closest_node_in_tree.position_.z_-x_in.position_.z_);
    
    return Node(Vector3d(x_new, y_new, z_new));
}

Node RTRRTStarClass::Uniform()
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
    
    return Node(Vector3d(x_new, y_new, z_new));
}

Node RTRRTStarClass::Ellipsoid(Node x_a, Node x_b)
{
    Eigen::Vector3d x_a_eigen = Eigen::Vector3d(x_a.position_.x_, x_a.position_.y_, x_a.position_.z_);
    Eigen::Vector3d x_b_eigen = Eigen::Vector3d(x_b.position_.x_, x_b.position_.y_, x_b.position_.z_);
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

    return Node(DrawWithinEllipsoid(covariance, x_center));
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

Node& RTRRTStarClass::GetClosestNodeInTree(Node x_in)
{
    Node& closest_node = x_in;
    double min_distance = std::numeric_limits<double>::infinity();
    double tree_node_distance;
    for (auto& tree_node : T)
    {
        tree_node_distance = EuclidianDistance3d(tree_node, x_in);
        if(tree_node_distance < min_distance)
        {
            min_distance = tree_node_distance;
            closest_node = tree_node;
        }
    }
    
    return closest_node;
}

bool RTRRTStarClass::CheckIfCollisionFreeLineBetween(Node x_a, Node x_b)
{
    if(Xi_obs.octomap_space_ == NULL)
    {
        return true;
    }
    else
    {
        octomap::point3d start(x_a.position_.x_,
                                x_a.position_.y_,
                                x_a.position_.z_);
        octomap::point3d direction(x_b.position_.x_ - x_a.position_.x_,
                                    x_b.position_.y_ - x_a.position_.y_,
                                    x_b.position_.z_ - x_a.position_.z_);
        octomap::point3d cell_hit_by_ray;
        

        return Xi_obs.octomap_space_->castRay(start, direction, cell_hit_by_ray);
    }
}

double RTRRTStarClass::EuclidianDistance3d(Node a, Node b)
{
    return std::sqrt( std::pow((a.position_.x_-b.position_.x_), 2) + std::pow((a.position_.y_-b.position_.y_), 2) + std::pow((a.position_.z_-b.position_.z_), 2));
}

double RTRRTStarClass::UniformRandomNumberBetween(double a, double b)
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



}  // namespace js_trajectory_planning_node
