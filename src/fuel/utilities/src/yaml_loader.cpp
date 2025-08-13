#include "utilities/yaml_loader.h"
#include <Eigen/Dense>
#include <vector>

// functions for converting between YAML::Node and Eigen::VectorXd
namespace YAML
{
  template <>
  struct convert<Eigen::VectorXd>
  {
    static YAML::Node encode(const Eigen::VectorXd &eigen_vector)
    {
      YAML::Node YAML_node;
      int idx;
      for (idx = 0; idx < eigen_vector.rows(); idx++)
        YAML_node.push_back(eigen_vector(idx));
      return YAML_node;
    }

    static bool decode(const YAML::Node &YAML_node, Eigen::VectorXd &eigen_vector)
    {
      if (!YAML_node.IsSequence())
        return false;
      eigen_vector.resize(YAML_node.size());
      int idx;
      for (idx = 0; idx < YAML_node.size(); idx++)
        eigen_vector(idx) = YAML_node[idx].as<double>();
      return true;
    }
  };
}

namespace utilities
{
  AllParams loadParamsFromYaml(const std::string &file_path)
  {
    AllParams params;
    YAML::Node config = YAML::LoadFile(file_path);

    loadSysParams(params, config);
    loadConstraints(params, config); // TODO
    loadMPCParams(params, config);
    loadGovernorParams(params, config);

    return params;
  }

  void loadSysParams(AllParams &params, const YAML::Node &config)
  {
    params.sys_params.g = config["g"].as<double>();
    params.sys_params.agent_mass = config["agent_mass"].as<double>();
    params.sys_params.agent_size = config["agent_size"].as<double>();
    params.sys_params.state_dim = config["state_dim"].as<int>();
    params.sys_params.ctrl_input_dim = config["ctrl_input_dim"].as<int>();
    params.sys_params.state_ctrl_input_dim = params.sys_params.state_dim + params.sys_params.ctrl_input_dim;
    params.sys_params.equilib_dim = config["equilib_dim"].as<int>();
    // TODO
    params.sys_params.A_matirx; // the A in x_{k+1} = A * x_k + B * u_k
    params.sys_params.B_matrix;
  }

  void loadConstraints(AllParams &params, const YAML::Node &config)
  {

    params.constraints.ctrl_input_min = config["ctrl_input_min"].as<Eigen::VectorXd>();
    params.constraints.ctrl_input_max = config["ctrl_input_max"].as<Eigen::VectorXd>();
    params.constraints.state_max = config["state_max"].as<Eigen::VectorXd>();
    params.constraints.state_min = -params.constraints.state_max;

    for (auto &obstacle : config["obstacles"])
    {
      Eigen::VectorXd obstacle_eigen = obstacle.as<Eigen::VectorXd>();
      params.constraints.obstacle_pos.push_back(obstacle_eigen);
    }
    for (auto &size : config["obstacle_sizes"])
      params.constraints.obstacle_sizes.push_back(size.as<double>());
  }

  void loadMPCParams(AllParams &params, const YAML::Node &config)
  {
    params.MPC_params.N = config["N"].as<int>();
    Eigen::VectorXd Q_diag = config["Q_MPC_diagonal"].as<Eigen::VectorXd>();
    params.MPC_params.Q_matrix = Q_diag.asDiagonal();
    Eigen::VectorXd R_diag = config["R_MPC_diagonal"].as<Eigen::VectorXd>();
    params.MPC_params.R_matrix = R_diag.asDiagonal();
    computeMPCParams(params);
  }

  // this function calculatres all parameters that aren't directly loaded from the YAML file
  void computeMPCParams(AllParams &params)
  {
    // shorthands
    int state_dim = params.sys_params.state_dim;

    // declarations
    int predicted_step;
    Eigen::VectorXd state_ctrl_input_max;
    Eigen::VectorXd state_ctrl_input_min;
    int row_start_idx;
    int col_start_idx;
    int row_idx;
    int col_idx;
    double state_space_constr_element;
    int state_space_constr_idx;

    params.MPC_params.decision_var_init = Eigen::VectorXd::Zero(params.MPC_params.decision_var_dim);
    params.MPC_params.decision_var_dim = params.MPC_params.N * params.sys_params.state_ctrl_input_dim + state_dim;

    // solveLQR(params);

    params.MPC_params.K_matrix.resize(state_dim, params.sys_params.ctrl_input_dim);
    params.MPC_params.K_matrix << 0, 0, 1.1473, 0, 0, 0.3732, 0, 0, 0, 0, 5.6905, 0, 0, -3.3565, 0, 8.3151, 0, 0, 5.6905, 0, 0, 3.3565, 0, 0, 0, 8.3151, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.9039;

    params.MPC_params.P_matrix.resize(state_dim, state_dim);
    params.MPC_params.P_matrix << 58.9833, -0.0000, 0.0000, 11.9460, -0.0000, 0.0000, 0.0000, 10.7492, 0.0000, -0.0000, 58.9833, -0.0000, -0.0000, 11.9460, -0.0000, -10.7492, -0.0000, 0.0000, 0.0000, -0.0000, 32.5243, 0.0000, -0.0000, 1.1629, 0.0000, 0.0000, -0.0000, 11.9460, -0.0000, 0.0000, 5.6031, -0.0000, -0.0000, 0.0000, 5.6642, -0.0000, -0.0000, 11.9460, -0.0000, -0.0000, 5.6031, -0.0000, -5.6642, -0.0000, 0.0000, 0.0000, -0.0000, 1.1629, -0.0000, -0.0000, 0.5701, 0.0000, -0.0000, 0.0000, 0.0000, -10.7492, 0.0000, 0.0000, -5.6642, 0.0000, 13.4176, 0.0000, -0.0000, 10.7492, -0.0000, 0.0000, 5.6642, -0.0000, -0.0000, 0.0000, 13.4176, -0.0000, 0.0000, 0.0000, -0.0000, -0.0000, 0.0000, 0.0000, -0.0000, -0.0000, 6.4039;

    // decision variable upper bound
    state_ctrl_input_max.resize(params.sys_params.state_ctrl_input_dim);
    state_ctrl_input_max
        << (params.constraints.state_max,
            params.constraints.ctrl_input_max);
    params.MPC_params.decision_var_upper_bound.resize(params.MPC_params.decision_var_dim);
    params.MPC_params.decision_var_upper_bound
        << (state_ctrl_input_max.colwise().replicate(params.MPC_params.N),
            params.constraints.state_max);

    // decision variable lower bound
    state_ctrl_input_min.resize(params.sys_params.state_ctrl_input_dim);
    state_ctrl_input_min
        << (params.constraints.state_min,
            params.constraints.ctrl_input_min);
    params.MPC_params.decision_var_lower_bound.resize(params.MPC_params.decision_var_dim);
    params.MPC_params.decision_var_lower_bound
        << (state_ctrl_input_min.colwise().replicate(params.MPC_params.N),
            params.constraints.state_min);

    // state-space constraint matrix
    params.MPC_params.state_space_constr = Eigen::MatrixXd::Zero(state_dim * (params.MPC_params.N + 1), params.MPC_params.decision_var_dim); // initialize it to zero, otherwise we will be accessing rows that don't exist
    params.MPC_params.state_space_constr.topLeftCorner(state_dim, state_dim).setIdentity();                                                  // the first timestep is special since it is the initial condition
    row_start_idx = state_dim;
    col_start_idx = 0;
    for (predicted_step = 1; predicted_step < params.MPC_params.N; predicted_step++)
    {
      params.MPC_params.state_space_constr.block(row_start_idx, col_start_idx,
                                                 state_dim, params.sys_params.state_ctrl_input_dim + state_dim)
          << (params.sys_params.A_matirx,
              params.sys_params.B_matrix,
              Eigen::MatrixXd::Identity(state_dim, state_dim)); // set a part of the matrix to [A, B, -I]
      row_start_idx += state_dim;
      col_start_idx += params.sys_params.state_ctrl_input_dim;
    }

    // convert state_space_constr to the format that IPOPT uses (i.e., a 1D array)
    // we basically just loop over the entire matrix and extract the row and column indices of nonzero elements
    params.MPC_params.state_space_constr_array.resize(params.MPC_params.state_space_constr.size());
    params.MPC_params.state_space_constr_row_indices.resize(params.MPC_params.state_space_constr.size());
    params.MPC_params.state_space_constr_col_indices.resize(params.MPC_params.state_space_constr.size());
    state_space_constr_idx = 0;
    for (row_idx = 0; row_idx < params.MPC_params.state_space_constr.rows(); row_idx++)
      for (col_idx = 0; col_idx < params.MPC_params.state_space_constr.rows(); col_idx++)
      {
        state_space_constr_element = params.MPC_params.state_space_constr(row_idx, col_idx);
        if (std::abs(state_space_constr_element) > params.zero_threshold)
        {
          params.MPC_params.state_space_constr_array(state_space_constr_idx) = state_space_constr_element;
          params.MPC_params.state_space_constr_row_indices(state_space_constr_idx) = row_idx;
          params.MPC_params.state_space_constr_col_indices(state_space_constr_idx) = col_idx;
          state_space_constr_idx++;
        }
      }

    // // TODO implement stuff for quadratic_weight (or just make a loop for computing the cost function)
    // params.MPC_params.quadratic_weight;
  }

  // TODO solver for the discrete Riccati equation
  void solveLQR(AllParams &params, // params will be modified
                double tolerance,
                int max_iter)
  {
    Eigen::MatrixXd P_next;
    int iter;
    Eigen::MatrixXd temp_term;
    Eigen::MatrixXd A = params.sys_params.A_matirx;
    Eigen::MatrixXd B = params.sys_params.B_matrix;
    Eigen::MatrixXd Q = params.MPC_params.Q_matrix;
    Eigen::MatrixXd R = params.MPC_params.R_matrix;
    Eigen::MatrixXd &P = params.MPC_params.P_matrix;
    Eigen::MatrixXd &K = params.MPC_params.K_matrix;
    // for (iter = 0; iter < max_iter; iter++)
    // {
    //   temp_term = B.transpose() * P * B;
    //   temp_term = (R + temp_term).inverse();
    //   K = temp_term * B.transpose() * P * A;
    //   P_next = A.transpose() * P * A - A.transpose() * P * B * K + Q; // TODO validate this formula (this was written by ChatGPT)
    //   if ((P_next - P).maxCoeff() < tolerance)
    //     break;
    //   P = P_next;
    // }
    P = Eigen::MatrixXd::Zero(params.sys_params.state_dim, params.sys_params.state_dim);
    K = Eigen::MatrixXd::Zero(params.sys_params.state_dim, params.sys_params.ctrl_input_dim);
  }

  void loadGovernorParams(AllParams &params, const YAML::Node &config)
  {
    params.governor_params.scale_factor_thrust = config["scale_factor_thrust"].as<double>();
    params.governor_params.scale_factor_obstacle = config["scale_factor_obstacle"].as<double>();
    params.governor_params.interpolation_factor = config["interpolation_factor"].as<double>();
    params.governor_params.max_iter_PathFG = config["max_iter_PathFG"].as<int>();
    params.governor_params.lower_dsm_bound_PathFG = config["lower_dsm_bound_PathFG"].as<double>();
    params.governor_params.upper_dsm_bound_PathFG = config["upper_dsm_bound_PathFG"].as<double>();
  }
} // namespace utilities
