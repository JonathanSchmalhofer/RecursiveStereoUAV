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
    // Todo: get closest node arg min dist(x,x_rand)
    if(true) // Todo: check if line(x_closest,x_rand) is in freespace Xi_free
    {
        FindNodesNear();
        if(true) // Todo: check line 6 Algorithm 2
        {
            AddNodeToTree();
            // Todo: Push x_rand to the first of Q_r
        }
        else
        {
            // Todo: Push x_closest to the first of Q_r
        }
        RewireRandomNodes();
    }
    RewireFromTreeRoot();
    ++counter_expansions_and_rewiring_;
}

void RTRRTStarClass::RewireRandomNodes()
{
}

void RTRRTStarClass::RewireFromTreeRoot()
{
}

void RTRRTStarClass::FindNodesNear()
{
}

void RTRRTStarClass::AddNodeToTree()
{
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
    
    double a = abs_direction[2];
    double b = abs_direction[1];
    double c = abs_direction[0];
    
    Eigen::Matrix< double, 3, 1> v;
    v << a, a, b;
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
