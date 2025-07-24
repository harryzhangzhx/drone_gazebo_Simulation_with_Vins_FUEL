#include "mpc.h"

MPC::MPC(const utilities::AllParams &params) : params_(params) {}

Eigen::VectorXd MPC::getMPCCommand(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s)
{
  Eigen::VectorXd vector;
  return vector;
}
