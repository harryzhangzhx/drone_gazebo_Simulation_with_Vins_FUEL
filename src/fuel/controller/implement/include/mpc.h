#pragma once
#include <vector>
#include <Eigen/Dense>
#include "utilities/common_types.h"

class MPC
{
public:
  MPC(const utilities::AllParams &params);
  Eigen::VectorXd getMPCCommand(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s);

private:
  utilities::AllParams params_;
};
