#include "dsm.h"
#include <algorithm>
#include <cmath>
#include <iostream>

DSM::DSM(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
    : agent_size_(params.sys_params.agent_size),
      ctrl_input_max_(params.constraints.ctrl_input_max),
      ctrl_input_min_(params.constraints.ctrl_input_min),
      obstacle_pos_(params.constraints.obstacle_pos),
      obstacle_sizes_(params.constraints.obstacle_sizes),
      scale_factor_thrust_(params.governor_params.scale_factor_thrust),
      scale_factor_obstacle_(params.governor_params.scale_factor_obstacle),
      K_matrix_(params.MPC_params.K_matrix),
      P_matrix_(params.MPC_params.P_matrix),
      voxel_map_ptr_(voxel_map_ptr)
{
}

/**
 * @brief calculate terminal dynamic safety margin
 *
 * @param current_state position, position_dot, roll, pitch, yaw
 * @param ref_state the target reference for position, position_dot, roll, pitch, yaw
 * @return double
 */
double DSM::getDSM(const Eigen::VectorXd &current_state,
                   const Eigen::VectorXd &ref_state)
{
  current_state_ = current_state;
  ref_state_ = ref_state;

  lyapunov_val_ = getLyapunovValue(current_state, ref_state);

  auto DSM_thrust = scale_factor_thrust_ * (thrustThreshold() - lyapunov_val_);

  auto DSM_obstacle = scale_factor_obstacle_ * (obstacleThreshold() - (current_state - ref_state).norm());
  auto DSM_angular_rate = angularRateThreshold() - lyapunov_val_;

  auto dsm_min = std::min({DSM_thrust, DSM_obstacle, DSM_angular_rate});
  return dsm_min;
}

/**
 * @brief calculate terminal Lyapunov threshold
 *
 * @param current_state position, position_dot, roll, pitch, yaw
 * @param ref_state the reference (target)
 * @return double
 */
double DSM::getLyapunovThreshold(const Eigen::VectorXd &current_state,
                                 const Eigen::VectorXd &ref_state)
{
  current_state_ = current_state;
  ref_state_ = ref_state;
  double DSM_thrust = scale_factor_thrust_ * thrustThreshold();
  double DSM_obstacle = scale_factor_obstacle_ * obstacleThreshold();
  double DSM_angular_rate = angularRateThreshold();
  return std::min({DSM_thrust, DSM_obstacle, DSM_angular_rate});
}

/**
 * @brief Calculates the Lyapunov value of the current state with respect to the reference
 * @return Lyapunov value
 */
double DSM::getLyapunovValue(const Eigen::VectorXd &current_state,
                             const Eigen::VectorXd &ref_state) const
{
  Eigen::VectorXd diff = current_state - ref_state;
  double lyapunov_val = diff.transpose() * P_matrix_ * diff;
  // std::cout << "current_state: " << current_state.transpose() << std::endl;
  // std::cout << "ref_state: " << ref_state.transpose() << std::endl;
  // std::cout << "P_matrix_: " << P_matrix_ << std::endl;
  // std::cout << "lyapunov_val: " << lyapunov_val << std::endl;
  return lyapunov_val;
}

/**
 * @brief   Gets the dynamic safety margin of the thrust constraint, $\Delta^s_i$.
 * Compute threshold value separately for thrust_min and thrust_max:
 * gamma_T_max = (thrust_max - m * g)^2 / (K(1,1:9) * inv(P) * K(1, 1:9)');
 * gamma_T_min = (thrust_min - m * g)^2 / (K(1,1:9) * inv(P) * K(1, 1:9)');
 *
 * @return double the lyapunov threshold for thrust
 */
double DSM::thrustThreshold() const
{
  // std::cout << "K_matrix_: " << K_matrix_ << std::endl;
  // std::cout << "ref_state_: " << ref_state_.transpose() << std::endl;
  Eigen::VectorXd c_Tmin = K_matrix_.row(0).transpose();
  // std::cout << "c_Tmin: " << c_Tmin.transpose() << std::endl;
  double d_Tmin = -ctrl_input_min_(0) + K_matrix_.row(0) * ref_state_;
  // std::cout << "d_Tmin: " << d_Tmin << std::endl;

  double numerator = std::pow(-c_Tmin.transpose() * ref_state_ + d_Tmin, 2);
  ;
  double lyap_thresh_min = lyapunovThreshold(d_Tmin, c_Tmin);

  Eigen::VectorXd c_Tmax = -K_matrix_.row(0).transpose();
  double d_Tmax = ctrl_input_max_(0) - K_matrix_.row(0) * ref_state_;
  double lyap_thresh_max = lyapunovThreshold(d_Tmax, c_Tmax);

  // std::cout << "lyap_thresh_min: " << lyap_thresh_min << std::endl;
  // std::cout << "lyap_thresh_max: " << lyap_thresh_max << std::endl;

  double thrust_lyapunov_threshold = std::min(lyap_thresh_min, lyap_thresh_max);

  return thrust_lyapunov_threshold;
}

/**
 * @brief   Gets the dynamic safety margin of the obstacle constraint, $\Delta^o_i$. Also output the Lyapunov threshold value
 *
 * @return double the lyapunov threshold for obstacle constraints
 */
double DSM::obstacleThreshold() const
{
  Eigen::Vector3d pos = ref_state_.head<3>();
  double dist_to_obs = voxel_map_ptr_->getDistance(pos);
  // std::cout << "dist_to_obs: " << dist_to_obs << std::endl;
  // std::cout << "agent_size_: " << agent_size_ << std::endl;
  return dist_to_obs - agent_size_;
}

// /**
//  * @brief   Gets the dynamic safety margin of the obstacle constraint, $\Delta^o_i$. Also output the Lyapunov threshold value
//  *
//  * @return double the lyapunov threshold for obstacle constraints
//  */
// double DSM::obstacleThreshold() const
// {

//   // Initialize thresholds vector with size equal to number of obstacles
//   std::vector<double> thresholds(obstacle_pos_.size());

//   // Loop over all obstacles
//   for (size_t i = 0; i < obstacle_pos_.size(); ++i)
//   {
//     Eigen::Vector3d diff = ref_state_.head(3) - obstacle_pos_[i];
//     Eigen::VectorXd c(9);
//     c.head(3) = -diff;
//     c.tail(6).setZero();
//     c.normalize();

//     double d = c.head(3).dot(obstacle_pos_[i]) - obstacle_sizes_[i] - agent_size_;
//     double gamma = lyapunovThreshold(d, c);

//     thresholds[i] = gamma;
//   }

//   double obstacle_lyapunov_threshold = *std::min_element(thresholds.begin(), thresholds.end());

//   return obstacle_lyapunov_threshold;
// }

/**
 * @brief   Gets the dynamic safety margin of the angular rate constraint.
 *
 * @param x_bar_v referenced equlibrium state
 * @return double the lyapunov threshold for angular rates
 */
double DSM::angularRateThreshold() const
{
  // std::cout << "K_matrix_" << K_matrix_ << std::endl;
  std::vector<double> omega_max_thresholds(3);
  std::vector<double> omega_min_thresholds(3);

  // loop over omega_x, omega_y and omega_z
  for (int i = 0; i < 3; ++i)
  {
    Eigen::VectorXd c_max = -K_matrix_.row(i + 1).transpose();
    double d_max = -K_matrix_.row(i + 1) * ref_state_ + ctrl_input_max_(i + 1);
    omega_max_thresholds[i] = lyapunovThreshold(d_max, c_max);
    // std::cout << "omega_max_thresholds[" << i << "]: " << omega_max_thresholds[i] << std::endl;

    Eigen::VectorXd c_min = K_matrix_.row(i + 1).transpose();
    double d_min = K_matrix_.row(i + 1) * ref_state_ - ctrl_input_min_(i + 1);
    omega_min_thresholds[i] = lyapunovThreshold(d_min, c_min);
    // std::cout << "omega_min_thresholds[" << i << "]: " << omega_min_thresholds[i] << std::endl;
  }

  double omega_max_thresh = *std::min_element(omega_max_thresholds.begin(), omega_max_thresholds.end());
  double omega_min_thresh = *std::min_element(omega_min_thresholds.begin(), omega_min_thresholds.end());

  double angular_rate_lyapunov_threshold = std::min(omega_max_thresh, omega_min_thresh);

  // std::cout << "angular_rate_lyapunov_threshold: " << angular_rate_lyapunov_threshold << std::endl;

  return angular_rate_lyapunov_threshold;
}

/**
 * @brief Gets the Lyapunov threshold value, Gamma(ref_state), from the applied reference ref_state. The constraint is c' * x <= d(ref_state), and the Lyapunov function is getLyapunovValue(x, ref_state) = (x - ref_state)' * P * (x - ref_state) (with P > 0).
 *
 * @param d the d component of constraint: c.transpose() x <= d
 * @param c the c component of constraint: c.transpose() x <= d
 * @return Lyapunov threshold value
 */
double DSM::lyapunovThreshold(const double &d, const Eigen::VectorXd &c) const
{
  double numerator = std::pow(-c.transpose() * ref_state_ + d, 2);
  double denominator = c.transpose() * P_matrix_.inverse() * c;
  return numerator / denominator;
}