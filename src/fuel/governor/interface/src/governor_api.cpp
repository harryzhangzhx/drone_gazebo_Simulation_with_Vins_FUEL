#include "governor/governor_api.h"
#include "erg.h"
#include "pathfg.h"
#include <iostream>

Governor::Governor(const utilities::AllParams &params) : params_(params)
{
    pathFG_ptr_ = std::make_shared<PathFG>(params);
}

double Governor::getGovernorRef(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s)
{
    return pathFG_ptr_->computePathFGRef(path, x, prior_s);
}