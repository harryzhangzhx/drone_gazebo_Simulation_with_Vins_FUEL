#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ctrl_manager
{
    enum Status
    {
        IDLE,
        NEW_TARGET,
        EXECUTING,
        SAFETY_HOVERING,
        LANDING
    };

    struct OdomWorld
    {
        Eigen::Vector3d odom_pos;
        Eigen::Vector3d odom_vel;
        Eigen::Quaterniond odom_orient;
        double odom_yaw;
    };

}
