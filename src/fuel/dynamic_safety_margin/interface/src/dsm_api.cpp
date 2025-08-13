#include "dsm/dsm_api.h"
#include <iostream>
#include "dsm.h"

DynamicSafetyMargin::DynamicSafetyMargin(const utilities::AllParams &params, const std::shared_ptr<VoxelMap> voxel_map_ptr)
{
    dsm_ptr_ = std::make_shared<DSM>(params, voxel_map_ptr);
}

/**
 * @brief calculate the dynamic safety margin of the state with respect to reference
 *
 * @param current_state state (position, position_dot, roll, pitch, yaw)
 * @param ref_state the target reference for state
 * @return double value of dyanmic safety margin
 */
double DynamicSafetyMargin::getDSM(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state)
{
    double dsm_value = dsm_ptr_->getDSM(current_state, ref_state);
    return dsm_value;
}

/**
 * @brief calculate Lyapunov threshold of the state with respect to reference
 *
 * @param current_state position, position_dot, roll, pitch, yaw
 * @param ref_state the target reference for state
 * @return double value of Lyapunov threshold
 */
double DynamicSafetyMargin::getLyapunovThreshold(const Eigen::VectorXd &current_state, const Eigen::VectorXd &ref_state)
{
    dsm_ptr_->getLyapunovThreshold(current_state, ref_state);
}

double DynamicSafetyMargin::getLyapunovValue(const Eigen::VectorXd &current_state,
                                             const Eigen::VectorXd &ref_state) const
{
    dsm_ptr_->getLyapunovValue(current_state, ref_state);
}