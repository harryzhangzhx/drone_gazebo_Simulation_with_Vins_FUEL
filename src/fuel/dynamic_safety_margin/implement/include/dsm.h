#pragma once

#include <vector>
#include <Eigen/Dense>
#include "utilities/common_types.h"

class DSM
{
public:
    DSM(const utilities::AllParams &params);

    double getDSM(const Eigen::VectorXd &x, const Eigen::VectorXd &v);
    double getLyapunovThreshold(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &v);

private:
    double lyapunovValue() const;
    double thrustThreshold() const;
    double obstacleThreshold() const;
    double angularRateThreshold() const;
    double lyapunovThreshold(const double &d, const Eigen::VectorXd &c) const;

    // params
    double agent_size_;
    Eigen::VectorXd ctrl_input_max_;
    Eigen::VectorXd ctrl_input_min_;
    std::vector<Eigen::Vector3d> obstacle_pos_;
    std::vector<double> obstacle_sizes_;
    double kappa_s_;
    double kappa_o_;
    Eigen::MatrixXd K_matrix_;
    Eigen::MatrixXd P_matrix_;

    // DSM variables
    double lyapunov_val_; // lyapunov value of the state with respect to reference
    Eigen::VectorXd x_;   // state
    Eigen::VectorXd v_;   // reference
};
