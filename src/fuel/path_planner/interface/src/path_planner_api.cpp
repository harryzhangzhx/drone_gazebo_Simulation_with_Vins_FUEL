#include <iostream>
#include <path_planner/path_planner_api.h>
#include "astar2.h"
#include "rrtstar.h"
#include "potential_field.h"

// for the path planner functions, need to see how we are going to pass in the map

PathPlanner::PathPlanner(const utilities::AllParams &params, std::shared_ptr<VoxelMap> map) : map_(map)
{
    params_ = params;
    astar_ptr_ = std::make_shared<Astar>();
    astar_ptr_->init(map); // TODO
    std::cout << "Astar initialized" << std::endl;
    rrtstar_ptr_ = std::make_shared<RRTstar>(map);
    std::cout << "RRTstar initialized" << std::endl;
    potential_field_ptr_ = std::make_shared<PotentialField>(map);
    std::cout << "PotentialField initialized" << std::endl;
}

bool PathPlanner::getRRTPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path)
{
    if (rrtstar_ptr_->search(start_pt, end_pt) == 1)
    {
        path = rrtstar_ptr_->shortestPath_;
        std::cout << "getRRTPath succeed with path size: " << path.size() << std::endl;
        return true;
    }
    std::cout << "getRRTPath failed" << std::endl;
    return false;
}
bool PathPlanner::getPotentialFieldPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path)
{

    if (potential_field_ptr_->calcNavField(start_pt, end_pt) == 1)
    {
        path = potential_field_ptr_->path_;
        std::cout << "PotentialField succeed with path size: " << path.size() << std::endl;
        return true;
    }
    std::cout << "getPotentialFieldPath failed" << std::endl;
    return false;
}

bool PathPlanner::getAstarPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path)
{

    if (astar_ptr_->search(start_pt, end_pt) == 1)
    {
        path = astar_ptr_->getPath();
        return true;
    }
    return false;
}
void PathPlanner::setRRTStarParams(double g, double eta)
{
    // rrtstar_ptr_->g_ = g;
    // rrtstar_ptr_->eta_ = eta;
    rrtstar_ptr_->tuneConstants(g, eta);
}
void PathPlanner::setPotentialFieldParams(double smoothing_radius, double influence_margin, double static_safety_margin, double collision_radius)
{

    potential_field_ptr_->setRepulseVars(influence_margin, static_safety_margin, collision_radius);
    potential_field_ptr_->setSmoothingRadius(smoothing_radius);
}
// void PathPlanner::setAStarParams(){

//} // to be implemented

// namespace
