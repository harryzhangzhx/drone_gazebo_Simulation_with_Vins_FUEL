#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ctrl_manager
{
    enum State
    {
        IDLE,
        NEW_TARGET,
        EXECUTING,
        SAFETY_HOVERING,
        LANDING
    };

    struct OdomWorld
    {
        Eigen::Vector3d odom_pos_;
        Eigen::Vector3d odom_vel_;
        Eigen::Quaterniond odom_orient_;
        double odom_yaw_;
    };
}
