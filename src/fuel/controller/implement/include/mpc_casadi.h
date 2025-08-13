#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <casadi/casadi.hpp>

#include "utilities/common_types.h"
#include "utilities/voxel_map.h"
#include "dsm/dsm_api.h"

class MPCCasadi
{
public:
  MPCCasadi(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);

  Eigen::VectorXd getMPCCasadiCommand(const Eigen::VectorXd &current_state,
                                      const Eigen::VectorXd ref_state);

private:
  Eigen::VectorXd getLinearWeight(Eigen::VectorXd ref_state);

  utilities::AllParams params_;
  std::shared_ptr<DynamicSafetyMargin> dsm_ptr_;
  std::shared_ptr<VoxelMap> voxel_map_ptr_;
};
