#include "controller/controller_api.h"
#include "lqr.h"
#include "mpc.h"
#include <iostream>

Controller::Controller(const utilities::AllParams &params) : params_(params)
{
    MPC_ptr_ = std::make_shared<MPC>(params);
}

Eigen::VectorXd Controller::getMPCCommand(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s)
{
    return MPC_ptr_->getMPCCommand(path, x, prior_s);
}
