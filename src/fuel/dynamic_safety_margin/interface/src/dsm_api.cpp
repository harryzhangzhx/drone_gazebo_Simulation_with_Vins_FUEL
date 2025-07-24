#include "dsm/dsm_api.h"
#include <iostream>
#include "dsm.h"

DynamicSafetyMargin::DynamicSafetyMargin(const utilities::AllParams &params)
{
    dsm_ptr_ = std::make_shared<DSM>(params);
}

/**
 * @brief calculate the dynamic safety margin of the state with respect to reference
 *
 * @param x state
 * @param v reference (position + yaw)
 * @return double value of dyanmic safety margin
 */
double DynamicSafetyMargin::getDSM(const Eigen::VectorXd &x, const Eigen::VectorXd &v)
{
    dsm_ptr_->getDSM(x, v);
}

/**
 * @brief calculate Lyapunov threshold of the state with respect to reference
 *
 * @param x state
 * @param v reference (position + yaw)
 * @return double value of Lyapunov threshold
 */
double DynamicSafetyMargin::getLyapunovThreshold(const Eigen::VectorXd &x, const Eigen::VectorXd &v)
{
    dsm_ptr_->getLyapunovThreshold(x, v);
}