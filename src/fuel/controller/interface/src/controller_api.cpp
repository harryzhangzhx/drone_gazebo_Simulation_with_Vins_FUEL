#include "lqr.h"
#include "mpc.h"
#include "controller/controller_api.h"

Controller::Controller(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
{
  // MPC_ptr_ = std::make_shared<MPC>(params, voxel_map_ptr);
  LQR_ptr_ = std::make_shared<LQR>(params);
}

// Eigen::VectorXd Controller::getMPCCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd ref_state)
// {
//     return MPC_ptr_->getMPCCommand(current_state, ref_state);
// }

Eigen::VectorXd Controller::getLQRCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state)
{
  return LQR_ptr_->getLQRCommand(current_state, ref_state);
}
