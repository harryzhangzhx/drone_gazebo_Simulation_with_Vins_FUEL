#include "utilities/yaml_loader.h"
#include <Eigen/Dense>
#include <vector>

namespace utilities
{

  AllParams loadParamsFromYaml(const std::string &file_path)
  {
    AllParams params;
    YAML::Node config = YAML::LoadFile(file_path);

    loadSysParams(params, config);
    loadConstraints(params, config);
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
    params.sys_params.equilib_dim = config["equilib_dim"].as<int>();
    // TODO
    params.sys_params.A_matirx; // x_{i+1} = A * x_i + B * u_i
    params.sys_params.B_matrix;
  }

  void loadConstraints(AllParams &params, const YAML::Node &config)
  {
    params.constraints.ctrl_input_min.setZero(params.sys_params.ctrl_input_dim);
    params.constraints.ctrl_input_max.setZero(params.sys_params.ctrl_input_dim);
    params.constraints.state_min.setZero(params.sys_params.state_dim);
    params.constraints.state_max.setZero(params.sys_params.state_dim);

    params.constraints.ctrl_input_min << config["thrust_min"].as<double>(),
        -config["omega_x_max"].as<double>(),
        -config["omega_y_max"].as<double>(),
        -config["omega_z_max"].as<double>();

    params.constraints.ctrl_input_max << config["thrust_max"].as<double>(),
        config["omega_x_max"].as<double>(),
        config["omega_y_max"].as<double>(),
        config["omega_z_max"].as<double>();

    // TODO read state bounds
    params.constraints.state_min; // [x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw]
    params.constraints.state_max;

    for (const auto &obs : config["obstacles"])
    {
      Eigen::Vector3d obstacle;
      obstacle << obs[0].as<double>(), obs[1].as<double>(), obs[2].as<double>();
      params.constraints.obstacle_pos.push_back(obstacle);
    }

    for (const auto &size : config["obstacle_sizes"])
    {
      params.constraints.obstacle_sizes.push_back(size.as<double>());
    }
  }

  void loadMPCParams(AllParams &params, const YAML::Node &config)
  {
    params.MPC_params.N = config["N"].as<int>();

    std::vector<double> Q_diag_vec = config["Q_MPC_diagonal"].as<std::vector<double>>();
    Eigen::VectorXd Q_diag = Eigen::Map<Eigen::VectorXd>(Q_diag_vec.data(), Q_diag_vec.size());
    params.MPC_params.Q_matrix = Q_diag.asDiagonal();

    std::vector<double> R_diag_vec = config["R_MPC_diagonal"].as<std::vector<double>>();
    Eigen::VectorXd R_diag = Eigen::Map<Eigen::VectorXd>(R_diag_vec.data(), R_diag_vec.size());
    params.MPC_params.R_matrix = R_diag.asDiagonal();

    // TODO
    params.MPC_params.decision_var_dim;   // N * (ctrl_input_dim + state_dim) + state_dim;
    params.MPC_params.K_matrix;           // LQR gain matrix
    params.MPC_params.P_matrix;           // terminal cost matrix
    params.MPC_params.state_space_constr; // this enforces the constraint xi_{i+1} = A * xi_i + B * mu_i using state_space_constraint * X = [ x_0, 0, 0, ... ]
    params.MPC_params.quadratic_weight;   /* the weight matrix for the quadratic term in the OCP cost function (i.e., the H in 1/2 * X^T * H * X); has Q's and R's alternating on its diagonal, ending with a P in the lower right corner */
    params.MPC_params.decision_var_lower_bound;
    params.MPC_params.decision_var_upper_bound;
  }

  void loadGovernorParams(AllParams &params, const YAML::Node &config)
  {
    params.governor_params.kappa_s = config["kappa_s"].as<int>();
    params.governor_params.kappa_o = config["kappa_o"].as<int>();
    params.governor_params.kappa_PathFG = config["kappa_PathFG"].as<int>();
    params.governor_params.max_iter_PathFG = config["max_iter_PathFG"].as<int>();
    params.governor_params.lower_dsm_bound_PathFG = config["lower_dsm_bound_PathFG"].as<int>();
    params.governor_params.upper_dsm_bound_PathFG = config["upper_dsm_bound_PathFG"].as<int>();
  }

} // namespace utilities
