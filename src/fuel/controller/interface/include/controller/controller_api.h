#pragma once
#include <Eigen/Core>
#include <memory>
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"

class MPC;
class LQR;
class Controller
{
public:
  Controller(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
  Eigen::VectorXd getMPCCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state);
  Eigen::VectorXd getLQRCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state);

private:
  // std::shared_ptr<MPC> MPC_ptr_;
  std::shared_ptr<LQR> LQR_ptr_;
};
