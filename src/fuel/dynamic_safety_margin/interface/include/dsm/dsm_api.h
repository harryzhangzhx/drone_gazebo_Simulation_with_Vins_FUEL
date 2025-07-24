#pragma once
#include <Eigen/Dense>
#include <memory>
#include "utilities/common_types.h"

class DSM;

class DynamicSafetyMargin
{
public:
  DynamicSafetyMargin(const utilities::AllParams &params);
  double getDSM(const Eigen::VectorXd &x, const Eigen::VectorXd &v);
  double getLyapunovThreshold(const Eigen::VectorXd &x, const Eigen::VectorXd &v);

private:
  std::shared_ptr<DSM> dsm_ptr_;
};
