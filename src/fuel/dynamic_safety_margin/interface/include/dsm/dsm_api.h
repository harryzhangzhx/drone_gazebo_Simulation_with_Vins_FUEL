#pragma once
#include <Eigen/Core>
#include <memory>
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"

class DSM;

class DynamicSafetyMargin
{
public:
  DynamicSafetyMargin(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
  double getDSM(const Eigen::VectorXd &x, const Eigen::VectorXd &ref_state);
  double getLyapunovThreshold(const Eigen::VectorXd &x, const Eigen::VectorXd &ref_state);
  double getLyapunovValue(const Eigen::VectorXd &current_state,
                          const Eigen::VectorXd &ref_state) const;

private:
  std::shared_ptr<DSM> dsm_ptr_;
};
