#include "lqr.h"

LQR::LQR(const utilities::AllParams &params) : K_(params.MPC_params.K_matrix)
{
}

Eigen::VectorXd LQR::getLQRCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state)
{
  return -K_ * (current_state - ref_state);
}
