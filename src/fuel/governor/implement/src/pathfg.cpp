#include "pathfg.h"
#include <iostream>

PathFG::PathFG(const utilities::AllParams &params) : kappa_PathFG_(params.governor_params.kappa_PathFG),
                                                     max_iter_PathFG_(params.governor_params.max_iter_PathFG),
                                                     lower_dsm_bound_PathFG_(params.governor_params.lower_dsm_bound_PathFG),
                                                     upper_dsm_bound_PathFG_(params.governor_params.upper_dsm_bound_PathFG)
{
    dsm_ptr_ = std::make_shared<DynamicSafetyMargin>(params);
}

double PathFG::computePathFGRef(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s)
{
    // if the auxiliary reference is already the end of the path
    double path_size = static_cast<double>(path.size());
    if (prior_s == path_size)
    {
        return prior_s;
    }

    // if the end of the path is already feasible
    VectorXd v = getVPosition(path, path_size);
    double DSM_val = dsm_ptr_->getDSM(x, v);
    int dsm_feasibility = checkDSMFeasibility(DSM_val);
    if (dsm_feasibility == 1)
    {
        return path_size;
    }

    // interpolate the path to find a feasible auxiliary reference
    int lower_s = prior_s;
    int upper_s = path_size;
    double s = upper_s;
    int iter = 0;

    // bisection method
    while (dsm_feasibility != 0 && iter < max_iter_PathFG_)
    {
        if (dsm_feasibility == -1)
        {
            upper_s = s;
        }
        else
        {
            lower_s = s;
        }

        s = lower_s + kappa_PathFG_ * (upper_s - lower_s);
        v = getVPosition(path, s);

        DSM_val = dsm_ptr_->getDSM(x, v);
        dsm_feasibility = checkDSMFeasibility(DSM_val);

        iter++;
    }

    return s;
}

int PathFG::checkDSMFeasibility(double DSM_val) const
{
    if (DSM_val < lower_dsm_bound_PathFG_)
        return -1;
    else if (DSM_val > upper_dsm_bound_PathFG_)
        return 1;
    else
        return 0;
}

Eigen::Vector4d PathFG::getVPosition(const std::vector<Eigen::VectorXd> &path, double s)
{

    size_t lower_idx = static_cast<size_t>(std::floor(s));
    size_t upper_idx = static_cast<size_t>(std::ceil(s));

    Eigen::Vector3d vector_delta = path[upper_idx] - path[lower_idx];
    Eigen::Vector3d interpolated = path[lower_idx] + (s - lower_idx) * vector_delta;

    // Create a 4D vector and set yaw to 0.0
    Eigen::Vector4d v;
    v.head<3>() = interpolated;
    v(3) = 0.0;

    return v;
}