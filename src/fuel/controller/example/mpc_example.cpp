// #include "controller/controller_api.h"
// #include <iostream>

// int main()
// {
//   std::cout << "controller example is running\n";

//   utilities::AllParams params;

//   // continuous time system matrices for the outer loop controller
//   double g = params.sys_params.g;
//   Eigen::MatrixXd G(3, 3); // intermediate matrix
//   G << 0, g, 0,
//       -g, 0, 0,
//       0, 0, 0;
//   Eigen::MatrixXd A_continuous = Eigen::MatrixXd::Zero(9, 9);
//   A_continuous.block(0, 3, 3, 3).setIdentity();
//   A_continuous.block(3, 6, 3, 3) = G;
//   Eigen::MatrixXd B_continuous = Eigen::MatrixXd::Zero(9, 4);
//   B_continuous.block(6, 1, 3, 3).setIdentity();
//   B_continuous(5, 0) = 1 / params.sys_params.agent_mass;

//   // TODO conver A_continuous and B_continuous to discrete time matrices
//   // temporary solution: get the discrete time matrices from MATLAB
//   params.sys_params.A_matirx.resize(9, 9);
//   params.sys_params.A_matirx << 1, 0, 0, 0, 0, 0, 0, 0.0491, 0, // the cpp-tools formatter insists on putting this on the same line
//       0, 1, 0, 0, 0.1, 0, -0.0491, 0, 0,
//       0, 0, 1, 0, 0, 0.1, 0, 0, 0,
//       0, 0, 0, 1, 0, 0, 0, 0.981, 0,
//       0, 0, 0, 0, 1, 0, -0.981, 0, 0,
//       0, 0, 0, 0, 0, 1, 0, 0, 0,
//       0, 0, 0, 0, 0, 0, 1, 0, 0,
//       0, 0, 0, 0, 0, 0, 0, 1, 0,
//       0, 0, 0, 0, 0, 0, 0, 0, 1;
//   params.sys_params.B_matrix.resize(9, 4);
//   params.sys_params.B_matrix << 0, 0, 0.0016, 0,
//       0, -0.0016, 0, 0,
//       0.1562, 0, 0, 0,
//       0, 0, 0.0491, 0,
//       0, -0.0491, 0, 0,
//       3.1250, 0, 0, 0,
//       0, 0.1000, 0, 0,
//       0, 0, 0.1000, 0,
//       0, 0, 0, 0.1000;

//   // TODO MPC parameters
//   params.MPC_params.Q_matrix = (Eigen::VectorXd{10, 10, 10, 5, 5, 5, 1, 1, 1}).asDiagonal(); // doesn't work?
//   params.MPC_params.R_matrix = Eigen::VectorXd{0.1, 0.1, 0.1, 0.1}.asDiagonal();
//   params.MPC_params.N = 20;

//   // LQR paramters
//   Eigen::MatrixXd Q_LQR = 2 * params.MPC_params.Q_matrix;
//   Eigen::MatrixXd R_LQR = 0.5 * params.MPC_params.R_matrix;
//   // TODO compute LQR gain matrices

//   // TODO voxel map parameters
//   VoxelMapParam voxel_map_params;
//   std::shared_ptr<VoxelMapParam> voxel_map_params_ptr = std::make_shared<VoxelMapParam>(voxel_map_params);
//   voxel_map_params.map_origin_ << 0, 0, 0;
//   voxel_map_params.map_size_ << 20, 20, 20;
//   voxel_map_params.map_min_boundary_ << -10, -10, -10;
//   voxel_map_params.map_max_boundary_ << 10, 10, 10;
//   voxel_map_params.map_voxel_num_ << 20, 20, 20;

//   // TODO make a voxel map with a cube at the center
//   VoxelMap voxel_map;
//   std::shared_ptr<VoxelMap> voxel_map_ptr = std::make_shared<VoxelMap>(voxel_map);

//   // do the actual control and print the results
//   Controller MPC_controller(params, voxel_map_ptr);
//   Eigen::VectorXd start_state = {-9, -9, -9, 0, 0, 0, 0, 0, 0};
//   Eigen::VectorXd end_state = {9, 9, 9, 0, 0, 0, 0, 0, 0};
//   Eigen::VectorXd optimal_decision_var = MPC_controller.getMPCCommand(start_state, end_state);
//   int predicted_idx;
//   int pos_start_idx;
//   for (predicted_idx = 0; predicted_idx <= params.MPC_params.N; predicted_idx++)
//   {
//     pos_start_idx = predicted_idx * params.sys_params.state_ctrl_input_dim;
//     Eigen::RowVectorXd pos = optimal_decision_var.segment(pos_start_idx, 3).transpose();
//     std::cout << pos << "\n"; // print out a row vector so that it's easier to read
//   }

//   return 0;
// }