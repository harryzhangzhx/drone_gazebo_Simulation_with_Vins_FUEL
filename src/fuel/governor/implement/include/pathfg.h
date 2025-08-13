#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "dsm/dsm_api.h"
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathFG
{
public:
    PathFG(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
    std::pair<double, Eigen::VectorXd> computePathFGRef(const std::vector<Eigen::Vector3d> &path, const Eigen::VectorXd &x, double &prior_s);

private:
    int checkDSMFeasibility(double DSM_val) const;
    Eigen::VectorXd getRefState(const std::vector<Eigen::Vector3d> &path, const double s);

    std::shared_ptr<DynamicSafetyMargin> dsm_ptr_;
    std::shared_ptr<VoxelMap> voxel_map_ptr_;
    double interpolation_factor_;
    int max_iter_PathFG_;
    double lower_dsm_bound_PathFG_;
    double upper_dsm_bound_PathFG_;
};
