// Copyleft 2025-2024 No One Inc.

#include "mpc.h"

MPC::MPC(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
    : params_(params),
      MPC_NLP_(new MPCNLP(params, voxel_map_ptr)),
      ipopt_app_(new Ipopt::IpoptApplication()),
      voxel_map_ptr_(voxel_map_ptr)
{
  dsm_ptr_ = std::make_shared<DynamicSafetyMargin>(params, voxel_map_ptr);

  // initialize the IPOPT application and use an approximation for the Hessian
  Ipopt::ApplicationReturnStatus status = ipopt_app_->Initialize();
  if (status != Ipopt::Solve_Succeeded)
    std::cerr << "! MPC CONTROLLER: ERROR WHILE INITIALIZING IPOPT\n";
  ipopt_app_->Options()->SetStringValue("hessian_approximation", "limited-memory");
}

Eigen::VectorXd MPC::getMPCCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd ref_state)
{
  // TODO remove the current state from the decision variable, which would reduce 9 dimensions from the optimization problem and potentially increase performance
  // TODO also implement using Fatrop (probably using the C++ interface, but also maybe try out the CasADi interface)
  // TODO maybe try libmpc++ (uses NLOPT)?
  // TODO save the params_ member variable by reference (i.e., utilities::AllParams& params_)
  // TODO use IPOPT without hard-coded paths

  // set the reference state and linear weight vector for the optimization problem
  MPC_NLP_->ref_state_ = ref_state;
  MPC_NLP_->current_state_ = current_state;
  MPC_NLP_->linear_weight_ = getLinearWeight(ref_state);

  // the Lyapunov threshold value is used for computing the terminal set constraint
  MPC_NLP_->lyapunov_threshold_ = dsm_ptr_->getLyapunovThreshold(current_state, ref_state);

  // do the actual optimization
  ipopt_app_->OptimizeTNLP(MPC_NLP_);

  return MPC_NLP_->optimal_decision_var_;
};

// produces the weight vector for the linear term of the cost function
// takes the form of [Q * ref_state, 0, Q * ref_state, 0,..., P * ref_state]
Eigen::VectorXd MPC::getLinearWeight(Eigen::VectorXd ref_state)
{
  // declarations
  Eigen::VectorXd linear_weight;
  Eigen::VectorXd Q_ref_state_and_zeros; // [Q * ref_state, 0]

  Q_ref_state_and_zeros << (params_.MPC_params.Q_matrix * ref_state,              // Q * ref_state
                            Eigen::VectorXd::Zero(params_.sys_params.state_dim)); // 0

  linear_weight << (Q_ref_state_and_zeros.colwise().replicate(params_.MPC_params.N), // [Q * ref_state, 0,...]
                    params_.MPC_params.P_matrix * ref_state);                        // P * ref_state

  return linear_weight;
}

// --- FUNCTIONS FOR IPOPT ---

// NLP stands for nonlinear program (a fancy way of calling the optimization problem);
MPCNLP::MPCNLP(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
    : params_(params),
      voxel_map_ptr_(voxel_map_ptr),
      optimal_decision_var_(params.MPC_params.decision_var_init) // for warm starting
{
  dsm_ptr_ = std::make_shared<DynamicSafetyMargin>(params, voxel_map_ptr);
}

// general information about the NLP
bool MPCNLP::get_nlp_info(Ipopt::Index &decision_var_dim,
                          Ipopt::Index &num_constraints,
                          Ipopt::Index &num_nonzero_jacobian,
                          Ipopt::Index &num_nonzero_hessian,
                          Ipopt::TNLP::IndexStyleEnum &idx_style)
{
  decision_var_dim = params_.MPC_params.decision_var_dim;

  // TODO move the following code outside

  // total number of linear, nonlinear, inequality and equality constraints
  int num_state_space_constraints = params_.sys_params.state_dim * params_.MPC_params.N; // total number of the xi_{k+1} = A * xi_k + B * mu_k constraints
  int num_init_condition_constraints = params_.sys_params.state_dim;                     // the xi_0 = x_0 constraint
  int num_nonlinear_constraints = params_.MPC_params.N;                                  // obstacle constraints (N - 1) and terminal set constraint (1)
  num_constraints = num_state_space_constraints + num_init_condition_constraints + num_nonlinear_constraints;

  // specify how sparse these matrices are (used by IPOPT)
  int num_nonzero_jacobian_obstacle = (params_.MPC_params.N - 1) * 3; // number of nonzero elements in the Jacobian that are from obstacle constraints
  int num_nonzero_jacobian_terminal_set = params_.sys_params.state_dim;
  int num_nonzero_jacobian_state_space = params_.MPC_params.state_space_constr_array.size();
  num_nonzero_jacobian = num_nonzero_jacobian_obstacle + num_nonzero_jacobian_terminal_set + num_nonzero_jacobian_state_space; // number of nonzero elements in the Jacobian of the constraints
  num_nonzero_hessian = 0;                                                                                                     // TODO number of nonzero elements in the lower left corner of the Hessian of the Lagrangian (leave as 0 for now since IPOPT can compute it using quasi-Newton methods)

  idx_style = Ipopt::TNLP::C_STYLE; // let indices start with 0 (as opposed to starting with 1 like in MATLAB or Fortran)

  // no error has occurred
  return true;
}

// bounds for the decision variable and constraints
bool MPCNLP::get_bounds_info(Ipopt::Index decision_var_dim,
                             Ipopt::Number *decision_var_lower_bound,
                             Ipopt::Number *decision_var_upper_bound,
                             Ipopt::Index num_constraints,
                             Ipopt::Number *constraints_lower_bound,
                             Ipopt::Number *constraints_upper_bound)
{
  // bounds for the decision variable (.data() gets the underlying C array from a std::vector)
  decision_var_lower_bound = params_.MPC_params.decision_var_lower_bound.data();
  decision_var_upper_bound = params_.MPC_params.decision_var_upper_bound.data();

  // TODO move this outside and put in the correct bounds for the equality constraints (note that the bounds for the initial condition isn't 0)
  // bounds for the constraints
  int constraint_idx;
  for (constraint_idx = 0; constraint_idx < num_constraints; constraint_idx++)
  {
    constraints_lower_bound[constraint_idx] = -2e19; // this will treated as negative infinity by IPOPT
    constraints_upper_bound[constraint_idx] = 0;     // our constraints are of the form g(x) <= 0
  }

  // no error has occurred
  return true;
}

// initialize the decision variable using POSS, which stands for "padded one-step shift";
// taken from "A Terminal State Feasibility Governor for Real-Time Nonlinear Model Predictive Control over Arbitrary Horizons,"
// it basically modifies the optimal control inputs and predicted states from the last timestep so that they act as better warm starts
bool MPCNLP::get_starting_point(Ipopt::Index decision_var_dim,
                                bool init_decision_var_flag,
                                Ipopt::Number *decision_var_init,
                                bool init_bound_multipliers_flag,
                                Ipopt::Number *bound_multipliers_lower_init,
                                Ipopt::Number *bound_multipliers_upper_init,
                                Ipopt::Index num_constraints,
                                bool init_constraints_multiplier_flag,
                                Ipopt::Number *constraints_multiplier_init)
{
  // under the default options, IPOPT shouldn't ask for initialized bound or constraint multipliers
  if (init_bound_multipliers_flag || init_constraints_multiplier_flag)
    return false; // send an error

  // use the LQR controller to approximate a solution to the optimization problem
  // NOTE: optimal_decision_var_ is the optimal solution from the last timestep
  Eigen::VectorXd prev_terminal_state = optimal_decision_var_.tail(params_.sys_params.state_dim);
  Eigen::VectorXd u_LQR = -params_.MPC_params.K_matrix * (prev_terminal_state - ref_state_);
  Eigen::VectorXd padded_terminal_state = params_.sys_params.A_matirx * prev_terminal_state + params_.sys_params.A_matirx * u_LQR;

  // discard the first control input and state in prev_decision_var and append padded_terminal_state
  Eigen::VectorXd decision_var_init_eigen;
  decision_var_init_eigen << (optimal_decision_var_.tail(params_.MPC_params.decision_var_dim -
                                                         params_.sys_params.state_ctrl_input_dim),
                              padded_terminal_state);

  // convert from Eigen::VectorXd to a C array
  Eigen::Map<Eigen::VectorXd>(decision_var_init, decision_var_dim) = decision_var_init_eigen;

  return true;
}

// the cost function;
// formula: 1/2 * X^T * quadratic_weight * X - linear_weight^T * X
bool MPCNLP::eval_f(Ipopt::Index decision_var_dim,
                    const Ipopt::Number *decision_var,
                    bool new_decision_var_flag, // can be ignored in our case
                    Ipopt::Number &cost)
{
  // convert from C array to Eigen::VectorXd
  // even though Ipopt::Number is typedef'ed as double, decision_var still needs to be explicitly casted to double*
  Eigen::Map<Eigen::VectorXd> decision_var_eigen((double *)decision_var, decision_var_dim);
  double quadratic_term = 0.5 * decision_var_eigen.transpose() * params_.MPC_params.quadratic_weight * decision_var_eigen;
  double linear_term = linear_weight_.transpose() * decision_var_eigen;
  cost = quadratic_term - linear_term;
  return true;
}

// the gradient of the cost function;
// formula: quadratic_weight * X - linear_weight
bool MPCNLP::eval_grad_f(Ipopt::Index decision_var_dim,
                         const Ipopt::Number *decision_var,
                         bool new_decision_var_flag,
                         Ipopt::Number *cost_gradient)
{
  Eigen::Map<Eigen::VectorXd> decision_var_eigen((double *)decision_var, decision_var_dim);
  Eigen::Map<Eigen::VectorXd>(cost_gradient, decision_var_dim) = params_.MPC_params.quadratic_weight * decision_var_eigen - linear_weight_;
  return true;
}

// constraint violation;
// includes the obstacle, terminal set, initial condition and state-space model constraints (in that order);
// this should be below 0 for the solution to be feasible
bool MPCNLP::eval_g(Ipopt::Index decision_var_dim,
                    const Ipopt::Number *decision_var,
                    bool new_decision_var_flag,
                    Ipopt::Index num_constraints,
                    Ipopt::Number *constraints_violation)
{
  // declarations
  int predicted_step;
  int pos_start_idx;
  double dist_from_obstacle;

  // conversion to Eigen::VectorXd
  Eigen::Map<Eigen::VectorXd> decision_var_eigen((double *)decision_var, decision_var_dim);

  // use the signed distance field as the obstacle constraint (1st to (N-1)th constraints)
  // NOTE: this constraint is not required for the initial state
  pos_start_idx = params_.sys_params.state_ctrl_input_dim;
  for (predicted_step = 1; predicted_step < params_.MPC_params.N; predicted_step++)
  {
    dist_from_obstacle = voxel_map_ptr_->getDistance((Eigen::Vector3d)decision_var_eigen.segment(pos_start_idx, 3)); // an explicit cast is requried to resolve an ADL ambiguity
    constraints_violation[predicted_step - 1] = dist_from_obstacle;
    pos_start_idx += params_.sys_params.state_ctrl_input_dim;
  }

  // terminal set constraint (Nth constaint)
  constraints_violation[params_.MPC_params.N - 1] = dsm_ptr_->getLyapunovValue(decision_var_eigen.tail(params_.sys_params.state_dim), ref_state_) - lyapunov_threshold_;

  // initial condition and state-space model constraints
  // i.e., xi_0 = x_0 and xi_{k+1} = A * xi_k + B * mu_k
  // NOTE: &constraints[N] points to the (N+1)th constraint
  // TODO turn this into a loop instead, which would probably be more efficient
  Eigen::Map<Eigen::VectorXd>(&constraints_violation[params_.MPC_params.N], params_.sys_params.state_dim) = decision_var_eigen.head(params_.sys_params.state_dim) - current_state_; // initial condition
  for (predicted_step = 0; predicted_step < params_.MPC_params.N; predicted_step++)
  {
    Eigen::Map<Eigen::VectorXd>(&constraints_violation[params_.MPC_params.N], decision_var_dim) = params_.MPC_params.state_space_constr * decision_var_eigen;
  }

  // no error has occurred
  return true;
}

bool MPCNLP::eval_jac_g(Ipopt::Index decision_var_dim,
                        const Ipopt::Number *decision_var,
                        bool new_decision_var_flag,
                        Ipopt::Index num_constraints,
                        Ipopt::Index num_nonzero_jacobian, // number of nonzero elements
                        Ipopt::Index *row_indices,         // array of the row indices of the nonzero elements
                        Ipopt::Index *col_indices,         // like row_indices, but for columns
                        Ipopt::Number *constraint_jacobian)
{
  // declarations
  int predicted_step;
  int pos_start_idx;
  int pos_idx;
  int constraint_jacobian_idx; // since the Jacobian is represented as a 1D array of its nonzero elements, this index is used for indexing into that array
  int state_idx;
  Eigen::Index state_space_matrix_row_idx;
  Eigen::Index state_space_matrix_col_idx;
  Eigen::Vector3d SDF_gradient;
  Eigen::VectorXd terminal_set_constraint_gradient;
  int row_idx;
  int col_idx;
  int state_space_constr_idx;
  int terminal_state_start_idx;
  int terminal_set_constraint_start_idx;
  int state_space_start_idx;

  if (constraint_jacobian == NULL)
  {
    // TODO move this outside (the sparsity structure doesn't change)
    // TODO make sure that this function is actually computing the correct values
    // constraint_jacobian will be NULL on the first call
    // here we will provide the sparsity structure of the Jacobian of the constraints
    // we do this by specifying the row and column indices of nonzero elements
    // the derivative of the ith constraint with respecet to the jth decision variable is on row i, column j

    // obstacle constraints
    for (predicted_step = 1; predicted_step < params_.MPC_params.N; predicted_step++)
    {
      pos_start_idx = params_.sys_params.state_ctrl_input_dim * predicted_step;
      for (pos_idx = 0; pos_idx < 3; pos_idx++)
      {
        constraint_jacobian_idx = (predicted_step - 1) * 3 + pos_idx;
        row_indices[constraint_jacobian_idx] = predicted_step - 1;
        col_indices[constraint_jacobian_idx] = pos_start_idx + pos_idx;
      }
    }

    // terminal set constraint
    terminal_set_constraint_start_idx = constraint_jacobian_idx + 1;                           // the terminal set constraint immediately follows the obstacle constraints (constraint_jacobian_idx was set to (N - 2) * 3 + 2 in the last iteration)
    terminal_state_start_idx = params_.sys_params.state_ctrl_input_dim * params_.MPC_params.N; // this is the starting index of the terminal state xi_N in the decision variable
    for (state_idx = 0; state_idx < params_.sys_params.state_dim; state_idx++)
    {
      row_indices[terminal_set_constraint_start_idx + state_idx] = params_.MPC_params.N; // the terminal set constraint is the Nth constraint
      col_indices[terminal_set_constraint_start_idx + state_idx] = terminal_set_constraint_start_idx + state_idx;
    }

    // state-space model constraint
    state_space_start_idx = terminal_set_constraint_start_idx + params_.sys_params.state_dim; // the state-space model constraint immediately follows the terminal set constraint
    std::copy(params_.MPC_params.state_space_constr_row_indices.begin(),
              params_.MPC_params.state_space_constr_row_indices.end(),
              &row_indices[state_space_start_idx]); // simply copy the precomputed values
    std::copy(params_.MPC_params.state_space_constr_col_indices.begin(),
              params_.MPC_params.state_space_constr_col_indices.end(),
              &col_indices[state_space_start_idx]);
  }
  else
  {
    // if constraint_jacobian isn't NULL, provide the Jacobian

    // as always, first convert to Eigen::VectorXd
    Eigen::Map<Eigen::VectorXd> decision_var_eigen((double *)decision_var, decision_var_dim);

    // obstacle constraints gradient
    constraint_jacobian_idx = 0;
    pos_start_idx = params_.sys_params.state_ctrl_input_dim;
    for (predicted_step = 1; predicted_step < params_.MPC_params.N; predicted_step++)
    {
      voxel_map_ptr_->getDistWithGrad(decision_var_eigen.segment(pos_start_idx, 3), SDF_gradient);
      SDF_gradient = -SDF_gradient;

      // we have to loop over the 3 dimensions in the gradient and assign it to the corresponding element in the Jacobian
      for (pos_idx = 0; pos_idx < 3; pos_idx++)
      {
        constraint_jacobian[constraint_jacobian_idx] = SDF_gradient(pos_idx);
        constraint_jacobian_idx++;
      }

      // the starting index of the next position vector is at an offset of state_dim + ctrl_input_dim
      pos_start_idx += params_.sys_params.state_ctrl_input_dim;
    }

    // terminal set constraint gradient
    // NOTE: constraint_jacobian_idx is already set to the correct value by the code for the obstacle constraints
    terminal_set_constraint_gradient = 2 * params_.MPC_params.P_matrix * (decision_var_eigen.tail(params_.sys_params.state_dim) - ref_state_);
    Eigen::Map<Eigen::VectorXd>(&constraint_jacobian[constraint_jacobian_idx], params_.sys_params.state_dim) = terminal_set_constraint_gradient; // convert to a C-style array

    // TODO this part can be moved outside since it doesn't change; also define the variables that this part needs
    // state-space model constraint
    // this literally just converts state_space_constraint into a 1D array that IPOPT can use
    constraint_jacobian_idx += params_.sys_params.state_dim;
    std::copy(params_.MPC_params.state_space_constr_array.begin(),
              params_.MPC_params.state_space_constr_array.end(),
              &constraint_jacobian[constraint_jacobian_idx]);
  }

  // no error occurred
  return true;
}

// TODO implement eval_h (optional--IPOPT can compute it using quasi-Newton)

// do stuff with the output (e.g., check for errors or print the output)
void MPCNLP::finalize_solution(Ipopt::SolverReturn status,
                               Ipopt::Index decision_var_dim,
                               const Ipopt::Number *optimal_decision_var,
                               const Ipopt::Number *optimal_bound_multipliers_lower,
                               const Ipopt::Number *optimal_bound_multipliers_upper,
                               Ipopt::Index num_constraints,
                               const Ipopt::Number *optimal_constraint,
                               const Ipopt::Number *optimal_constraint_multipliers,
                               Ipopt::Number optimal_cost,
                               const Ipopt::IpoptData *ip_data, /* not used */
                               Ipopt::IpoptCalculatedQuantities *ip_cq /* not used */)
{
  if (status != Ipopt::SolverReturn::SUCCESS)
    std::cerr << "! MPC CONTROLLER: SOMETHING WENT WRONG WITH THE OPTIMIZATION\n";

  // convert to Eigen::VectorXd and export the optimal solution
  Eigen::Map<Eigen::VectorXd> optimal_decision_var_((double *)optimal_decision_var, decision_var_dim);
}