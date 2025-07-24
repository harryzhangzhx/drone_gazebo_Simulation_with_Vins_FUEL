#include "dsm.h"
#include <algorithm>
#include <cmath>

DSM::DSM(const utilities::AllParams &params)
    : agent_size_(params.sys_params.agent_size),
      ctrl_input_max_(params.constraints.ctrl_input_max),
      ctrl_input_min_(params.constraints.ctrl_input_min),
      obstacle_pos_(params.constraints.obstacle_pos),
      obstacle_sizes_(params.constraints.obstacle_sizes),
      kappa_s_(params.governor_params.kappa_s),
      kappa_o_(params.governor_params.kappa_o),
      K_matrix_(params.MPC_params.K_matrix),
      P_matrix_(params.MPC_params.P_matrix)
{
}

/**
 * @brief calculate terminal dynamic safety margin
 *
 * @param x state of the agent
 * @param v the reference (target)
 * @return double
 */
double DSM::getDSM(const Eigen::VectorXd &x,
                   const Eigen::VectorXd &v)
{
    x_ = x;
    v_ = Eigen::VectorXd::Zero(9);
    v_.head(3) = v.head(3); // position
    v_(8) = v(3);           // yaw

    lyapunov_val_ = lyapunovValue();
    auto DSM_thrust = kappa_s_ * (thrustThreshold() - lyapunov_val_);
    auto DSM_obstacle = kappa_o_ * (obstacleThreshold() - lyapunov_val_);
    auto DSM_angular_rate = angularRateThreshold() - lyapunov_val_;

    return std::min({DSM_thrust, DSM_obstacle, DSM_angular_rate});
}

/**
 * @brief calculate terminal Lyapunov threshold
 *
 * @param x state of the agent
 * @param v the reference (target)
 * @return double
 */
double DSM::getLyapunovThreshold(const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &v)
{
    x_ = x;
    v_ = Eigen::VectorXd::Zero(9);
    v_.head(3) = v.head(3); // position
    v_(8) = v(3);           // yaw

    lyapunov_val_ = lyapunovValue();
    auto DSM_thrust = kappa_s_ * thrustThreshold();
    auto DSM_obstacle = kappa_o_ * obstacleThreshold();
    auto DSM_angular_rate = angularRateThreshold();

    return std::min({DSM_thrust, DSM_obstacle, DSM_angular_rate});
}

/**
 * @brief Calculates the Lyapunov value of the current state with respect to the reference
 *
 * @param x current state
 * @param v_ current applied reference
 * @return Lyapunov value
 */
double DSM::lyapunovValue() const
{
    Eigen::VectorXd diff = x_ - v_;
    return diff.transpose() * P_matrix_ * diff;
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
    Eigen::VectorXd c_Tmin = K_matrix_.row(0).transpose();
    double d_Tmin = -ctrl_input_min_(0) + K_matrix_.row(0) * v_;
    double lyap_thresh_min = lyapunovThreshold(d_Tmin, c_Tmin);

    Eigen::VectorXd c_Tmax = -K_matrix_.row(0).transpose();
    double d_Tmax = ctrl_input_max_(0) - K_matrix_.row(0) * v_;
    double lyap_thresh_max = lyapunovThreshold(d_Tmax, c_Tmax);

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

    // Initialize thresholds vector with size equal to number of obstacles
    std::vector<double> thresholds(obstacle_pos_.size());

    // Loop over all obstacles
    for (size_t i = 0; i < obstacle_pos_.size(); ++i)
    {
        Eigen::Vector3d diff = v_.head(3) - obstacle_pos_[i];
        Eigen::VectorXd c(9);
        c.head(3) = -diff;
        c.tail(6).setZero();
        c.normalize();

        double d = c.head(3).dot(obstacle_pos_[i]) - obstacle_sizes_[i] - agent_size_;
        double gamma = lyapunovThreshold(d, c);

        thresholds[i] = gamma;
    }

    double obstacle_lyapunov_threshold = *std::min_element(thresholds.begin(), thresholds.end());

    return obstacle_lyapunov_threshold;
}

/**
 * @brief   Gets the dynamic safety margin of the angular rate constraint.
 *
 * @param x_bar_v referenced equlibrium state
 * @return double the lyapunov threshold for angular rates
 */
double DSM::angularRateThreshold() const
{

    std::vector<double> omega_max_thresholds(3);
    std::vector<double> omega_min_thresholds(3);

    // loop over omega_x, omega_y and omega_z
    for (int i = 0; i < 3; ++i)
    {
        Eigen::VectorXd c_max = -K_matrix_.row(i + 1).transpose();
        double d_max = -K_matrix_.row(i + 1) * v_ + ctrl_input_max_(i + 1);
        omega_max_thresholds[i] = lyapunovThreshold(d_max, c_max);

        Eigen::VectorXd c_min = K_matrix_.row(i + 1).transpose();
        double d_min = K_matrix_.row(i + 1) * v_ - ctrl_input_min_(i + 1);
        omega_min_thresholds[i] = lyapunovThreshold(d_min, c_min);
    }

    double omega_max_thresh = *std::min_element(omega_max_thresholds.begin(), omega_max_thresholds.end());
    double omega_min_thresh = *std::min_element(omega_min_thresholds.begin(), omega_min_thresholds.end());

    double angular_rate_lyapunov_threshold = std::min(omega_max_thresh, omega_min_thresh);

    return angular_rate_lyapunov_threshold;
}

/**
 * @brief Gets the Lyapunov threshold value, Gamma(v), from the applied reference v. The constraint is c' * x <= d(v), and the Lyapunov function is lyapunovValue(x, v) = (x - v)' * P * (x - v) (with P > 0).
 *
 * @param d the d component of constraint: c.transpose() x <= d
 * @param c the c component of constraint: c.transpose() x <= d
 * @return Lyapunov threshold value
 */
double DSM::lyapunovThreshold(const double &d, const Eigen::VectorXd &c) const
{
    double numerator = std::pow(-c.transpose() * v_ + d, 2);
    double denominator = c.transpose() * P_matrix_.inverse() * c;
    return numerator / denominator;
}