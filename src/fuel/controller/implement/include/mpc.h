#pragma once

#include <Eigen/Dense>
#include <memory>
#include <coin-or/IpTNLP.hpp>
#include <coin-or/IpIpoptApplication.hpp>
#include <vector>

#include "utilities/common_types.h"
#include "utilities/voxel_map.h"
#include "dsm/dsm_api.h"

class MPCNLP;

class MPC
{
public:
  MPC(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
  Eigen::VectorXd getMPCCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd ref_state);

private:
  Eigen::VectorXd getLinearWeight(Eigen::VectorXd ref_state);

  utilities::AllParams params_;
  Ipopt::SmartPtr<MPCNLP> MPC_NLP_;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app_;
  std::shared_ptr<DynamicSafetyMargin> dsm_ptr_;
  std::shared_ptr<VoxelMap> voxel_map_ptr_;
};

class MPCNLP : public Ipopt::TNLP
{
public:
  MPCNLP(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);
  virtual bool get_nlp_info(Ipopt::Index &decision_var_dim,
                            Ipopt::Index &num_constraints,
                            Ipopt::Index &num_nonzero_jacobian,
                            Ipopt::Index &num_nonzero_hessian,
                            IndexStyleEnum &idx_style);
  virtual bool get_bounds_info(Ipopt::Index decision_var_dim,
                               Ipopt::Number *decision_var_lower_bound,
                               Ipopt::Number *decision_var_upper_bound,
                               Ipopt::Index num_constraints,
                               Ipopt::Number *constraints_lower_bound,
                               Ipopt::Number *constraints_upper_bound);
  virtual bool get_starting_point(Ipopt::Index decision_var_dim,
                                  bool init_decision_var_flag,
                                  Ipopt::Number *decision_var_init,
                                  bool init_bound_multipliers_flag,
                                  Ipopt::Number *bound_multipliers_lower_init,
                                  Ipopt::Number *bound_multipliers_upper_init,
                                  Ipopt::Index num_constraints,
                                  bool init_constraints_multiplier_flag,
                                  Ipopt::Number *constraints_multiplier_init);
  virtual bool eval_f(Ipopt::Index decision_var_dim,
                      const Ipopt::Number *decision_var,
                      bool new_decision_var_flag,
                      Ipopt::Number &cost);
  virtual bool eval_grad_f(Ipopt::Index decision_var_dim,
                           const Ipopt::Number *decision_var,
                           bool new_decision_var_flag,
                           Ipopt::Number *cost_gradient);
  virtual bool eval_g(Ipopt::Index decision_var_dim,
                      const Ipopt::Number *decision_var,
                      bool new_decision_var_flag,
                      Ipopt::Index num_constraints,
                      Ipopt::Number *constraints);
  virtual bool eval_jac_g(Ipopt::Index decision_var_dim,
                          const Ipopt::Number *decision_var,
                          bool new_decision_var_flag,
                          Ipopt::Index num_constraints,
                          Ipopt::Index num_nonzero_jacobian,
                          Ipopt::Index *row_indices,
                          Ipopt::Index *col_indices,
                          Ipopt::Number *constraint_jacobian);
  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index decision_var_dim,
                                 const Ipopt::Number *optimal_decision_var,
                                 const Ipopt::Number *optimal_bound_multipliers_lower,
                                 const Ipopt::Number *optimal_bound_multipliers_upper,
                                 Ipopt::Index num_constraints,
                                 const Ipopt::Number *optimal_constraint,
                                 const Ipopt::Number *optimal_constraint_multipliers,
                                 Ipopt::Number optimal_cost,
                                 const Ipopt::IpoptData *ip_data,
                                 Ipopt::IpoptCalculatedQuantities *ip_cq);

  // these need to be accessible from the MPC class
  Eigen::VectorXd ref_state_;
  Eigen::VectorXd current_state_;
  Eigen::VectorXd linear_weight_;
  double lyapunov_threshold_;
  Eigen::VectorXd optimal_decision_var_;

private:
  utilities::AllParams params_;
  std::shared_ptr<VoxelMap> voxel_map_ptr_;
  std::shared_ptr<DynamicSafetyMargin> dsm_ptr_;
};