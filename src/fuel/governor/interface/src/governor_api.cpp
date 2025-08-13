#include "governor/governor_api.h"
#include "erg.h"
#include "pathfg.h"
#include <iostream>

Governor::Governor(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
{
    pathFG_ptr_ = std::make_shared<PathFG>(params, voxel_map_ptr);
}

std::pair<double, Eigen::VectorXd> Governor::getGovernorRef(const std::vector<Eigen::Vector3d> &path, const Eigen::VectorXd &current_state, double &prior_s)
{
    return pathFG_ptr_->computePathFGRef(path, current_state, prior_s);
}