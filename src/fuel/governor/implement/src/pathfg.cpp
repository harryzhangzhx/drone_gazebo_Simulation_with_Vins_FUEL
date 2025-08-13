#include "pathfg.h"
#include <iostream>

PathFG::PathFG(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
    : interpolation_factor_(params.governor_params.interpolation_factor),
      max_iter_PathFG_(params.governor_params.max_iter_PathFG),
      lower_dsm_bound_PathFG_(params.governor_params.lower_dsm_bound_PathFG),
      upper_dsm_bound_PathFG_(params.governor_params.upper_dsm_bound_PathFG),
      voxel_map_ptr_(voxel_map_ptr)
{
  dsm_ptr_ = std::make_shared<DynamicSafetyMargin>(params, voxel_map_ptr);
}

std::pair<double, Eigen::VectorXd> PathFG::computePathFGRef(const std::vector<Eigen::Vector3d> &path, const Eigen::VectorXd &current_state, double &prior_s)
{
  // TODO reset s if the path is new
  std::cout << "prior_s:" << prior_s << std::endl;
  double path_size = static_cast<double>(path.size());
  std::cout << "path_size:" << path_size << std::endl;

  VectorXd ref_state = getRefState(path, path_size - 1); // the equilibrium state (position + yaw )at the reference

  // if the auxiliary reference is already the end of the path
  if (path_size - 1 - prior_s < 1e-2)
  {
    return {prior_s, ref_state};
  }

  // if the end of the path is already feasible
  double DSM_val = dsm_ptr_->getDSM(current_state, ref_state);
  int dsm_feasibility = checkDSMFeasibility(DSM_val);

  if (dsm_feasibility == 0)
  {
    return {path_size - 1, ref_state};
  }

  // interpolate the path to find a feasible auxiliary reference
  double lower_s = prior_s;
  double upper_s = path_size - 1;
  double s = upper_s;
  int iter = 0;

  // bisection method
  while (dsm_feasibility != 0 && iter < max_iter_PathFG_)
  {
    if (dsm_feasibility == -1)
    {
      upper_s = s;
    }
    else
    {
      lower_s = s;
    }
    s = lower_s + interpolation_factor_ * (upper_s - lower_s);
    ref_state = getRefState(path, s);
    DSM_val = dsm_ptr_->getDSM(current_state, ref_state);
    // std::cout << "DSM_val in the loop:" << DSM_val << std::endl;
    dsm_feasibility = checkDSMFeasibility(DSM_val);

    iter++;
  }

  std::cout << "pathFG completed" << std::endl;

  return {s, ref_state};
}

int PathFG::checkDSMFeasibility(double DSM_val) const
{
  if (DSM_val < lower_dsm_bound_PathFG_)
    return -1;
  else if (DSM_val > upper_dsm_bound_PathFG_)
    return 1;
  else
    return 0;
}

Eigen::VectorXd PathFG::getRefState(const std::vector<Eigen::Vector3d> &path, const double s)
{
  const double s_clamped = std::max(0.0, std::min(s, static_cast<double>(path.size() - 1)));
  size_t lower_idx = static_cast<size_t>(std::floor(s_clamped));
  size_t upper_idx = static_cast<size_t>(std::ceil(s_clamped));

  // TODO double check path index, make sure it's using the currect point (matlab vector starts from 1)
  Eigen::Vector3d vector_delta = path[upper_idx] - path[lower_idx];
  Eigen::Vector3d interpolated = path[lower_idx] + (s - lower_idx) * vector_delta;

  Eigen::VectorXd ref_state = Eigen::VectorXd::Zero(9);
  ref_state.head(3) = interpolated; // position

  return ref_state;
}
