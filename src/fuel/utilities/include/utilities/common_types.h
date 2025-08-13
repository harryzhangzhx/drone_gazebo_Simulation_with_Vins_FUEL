#pragma once
#include <Eigen/Dense>
#include <vector>
#include <numbers>

namespace utilities
{
  struct SysParams
  {
    double g = -9.81;          // gravitational acceleration
    double agent_mass = 32e-3; // [kg]
    double agent_size = 0.08;  // [m]
    int state_dim = 9;
    int ctrl_input_dim = 4;
    int state_ctrl_input_dim = 13;
    int equilib_dim = 4;
    Eigen::MatrixXd A_matirx; // x_{i+1} = A * x_i + B * u_i
    Eigen::MatrixXd B_matrix;
  };

  struct Constraints
  {
    Eigen::VectorXd ctrl_input_min; // [thrust, omega_x, omega_y, omega_z]
    Eigen::VectorXd ctrl_input_max;
    Eigen::VectorXd state_min; // [x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw]
    Eigen::VectorXd state_max;
    std::vector<Eigen::Vector3d> obstacle_pos;
    std::vector<double> obstacle_sizes;
  };

  struct MPCParams
  {
    int N;                                          // prediction horizon
    int decision_var_dim;                           // N * (ctrl_input_dim + state_dim) + state_dim;
    Eigen::VectorXd decision_var_init;              // initial decision varaible for the first time step
    Eigen::MatrixXd Q_matrix;                       // stage cost for the state
    Eigen::MatrixXd R_matrix;                       // stage cost for the control input
    Eigen::MatrixXd K_matrix;                       // LQR gain matrix
    Eigen::MatrixXd P_matrix;                       // terminal cost matrix
    Eigen::MatrixXd state_space_constr;             // this enforces the constraint xi_{i+1} = A * xi_i + B * mu_i using state_space_constr * X = [ x_0, 0, 0,...]
    Eigen::VectorXi state_space_constr_row_indices; // the row indices of the nonzero elements
    Eigen::VectorXi state_space_constr_col_indices; // the column indices of the nonzero elements
    Eigen::VectorXd state_space_constr_array;       // contains the nonzero elements of state_space_constr
    Eigen::MatrixXd quadratic_weight;               // the weight matrix for the quadratic term in the OCP cost function (i.e., the H in 1/2 * X^T * H * X); has Q's and R's alternating on its diagonal, ending with a P in the lower right corner
    Eigen::VectorXd decision_var_lower_bound;
    Eigen::VectorXd decision_var_upper_bound;
  };

  struct GovernorParams
  {
    double scale_factor_thrust;
    double scale_factor_obstacle;
    double interpolation_factor;
    int max_iter_PathFG;
    double lower_dsm_bound_PathFG;
    double upper_dsm_bound_PathFG;
  };

  struct AllParams
  {
    SysParams sys_params;
    Constraints constraints;
    MPCParams MPC_params;
    GovernorParams governor_params;
    double zero_threshold; // TODO relocate this somewhere better (if abs(num) < zero_threshold, then it is considered to be 0)
  };
} // namespace utilities