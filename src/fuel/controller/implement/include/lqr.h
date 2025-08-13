#pragma once
#include "utilities/common_types.h"

class LQR
{
public:
  LQR(const utilities::AllParams &params);
  Eigen::VectorXd getLQRCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state);

private:
  Eigen::MatrixXd K_;
};
