#pragma once

#include <atomic>
#include <Eigen/Core>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/next_pos_and_yaw.hpp"
#include "controller_manager_types.h"

namespace ctrl_manager
{
  class TopicManager
  {
  public:
    TopicManager(std::shared_ptr<rclcpp::Node> node);

    bool getNewTargetFlag() const;
    void setNewTargetFlag(const bool &new_target_flag);
    Eigen::Vector4d getNewTarget() const;
    ctrl_manager::OdomWorld getOdom() const;

  private:
    void nextPosAndYawCallback(const quadrotor_msgs::msg::NextPosAndYaw::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<quadrotor_msgs::msg::NextPosAndYaw>::SharedPtr next_pos_and_yaw_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::atomic<bool> new_target_flag_{false};
    mutable std::mutex task_mutex_;
    Eigen::Vector4d next_pos_and_yaw_;
    ctrl_manager::OdomWorld odom_world_;
  };

} // namespace ctrl_manager
