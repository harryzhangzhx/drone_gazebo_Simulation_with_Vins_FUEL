#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "dsm/dsm_api.h"
#include "utilities/common_types.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathFG
{
public:
    PathFG(const utilities::AllParams &params);
    double computePathFGRef(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s);

private:
    int checkDSMFeasibility(double DSM_val) const;
    Eigen::Vector4d getVPosition(const std::vector<Eigen::VectorXd> &path, double s);

    std::shared_ptr<DynamicSafetyMargin> dsm_ptr_;
    double kappa_PathFG_;
    int max_iter_PathFG_;
    double lower_dsm_bound_PathFG_;
    double upper_dsm_bound_PathFG_;
};
