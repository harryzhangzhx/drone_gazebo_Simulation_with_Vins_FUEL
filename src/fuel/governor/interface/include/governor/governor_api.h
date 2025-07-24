#pragma once
#include <Eigen/Dense>
#include <memory>
#include "utilities/common_types.h"

class PathFG;

class Governor
{
public:
  Governor(const utilities::AllParams &params);
  double getGovernorRef(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &x, const double &prior_s);

private:
  std::shared_ptr<PathFG> pathFG_ptr_;
  const utilities::AllParams &params_;
};
