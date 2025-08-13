#pragma once

#include <vector>
#include <Eigen/Core>
#include <memory>
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"

class DSM
{
public:
    DSM(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr);

    double getDSM(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state);
    double getLyapunovThreshold(const Eigen::VectorXd &current_state,
                                const Eigen::VectorXd &ref_state);
    double getLyapunovValue(const Eigen::VectorXd &current_state,
                            const Eigen::VectorXd &ref_state) const;

private:
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
    double scale_factor_thrust_;
    double scale_factor_obstacle_;
    Eigen::MatrixXd K_matrix_;
    Eigen::MatrixXd P_matrix_;

    // DSM variables
    double lyapunov_val_;           // lyapunov value of the state with respect to reference
    Eigen::VectorXd current_state_; // position, position_dot, roll, pitch, yaw
    Eigen::VectorXd ref_state_;     // reference for position, position_dot, roll, pitch, yaw

    // map
    std::shared_ptr<VoxelMap> voxel_map_ptr_;
};
