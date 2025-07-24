#pragma once
#include <Eigen/Dense>
#include <memory>
#include "utilities/common_types.h"

class MPC;

class Controller
{
public:
  Controller(const utilities::AllParams &params);
  Eigen::VectorXd getMPCCommand(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s);

private:
  std::shared_ptr<MPC> MPC_ptr_;
  const utilities::AllParams &params_;
};
