#pragma once
#include <Eigen/Core>
#include <memory>
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"

class PathFG;

class Governor
{
public:
  Governor(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
  std::pair<double, Eigen::VectorXd> getGovernorRef(const std::vector<Eigen::Vector3d> &path, const Eigen::VectorXd &x, double &prior_s);

private:
  std::shared_ptr<PathFG> pathFG_ptr_;
};
