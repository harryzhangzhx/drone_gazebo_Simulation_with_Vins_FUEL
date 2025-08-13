#include "controller_manager/topic_manager.h"
#include <iostream>
#include <functional>

namespace ctrl_manager
{

  TopicManager::TopicManager(std::shared_ptr<rclcpp::Node> node)
      : node_(node)
  {
    next_pos_and_yaw_sub_ = node_->create_subscription<quadrotor_msgs::msg::NextPosAndYaw>(
        "/planning/next_pos_and_yaw",
        rclcpp::QoS(10),
        std::bind(&TopicManager::nextPosAndYawCallback, this, std::placeholders::_1));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/state_ukf/odom",
        rclcpp::QoS(10),
        std::bind(&TopicManager::odometryCallback, this, std::placeholders::_1));

    thrust_omega_pub_ = node_->create_publisher<quadrotor_msgs::msg::ThrustAndOmega>(
        "/thrust_omega", rclcpp::QoS(10).reliable());
  }

  void TopicManager::nextPosAndYawCallback(const quadrotor_msgs::msg::NextPosAndYaw::SharedPtr msg)
  {
    // RCLCPP_INFO(node_->get_logger(), "Controller Manager: nextPosAndYawCallback received message:");
    // RCLCPP_INFO(node_->get_logger(), "Position: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);
    // RCLCPP_INFO(node_->get_logger(), "Yaw: %f", msg->yaw);
    new_target_flag_ = true;
    next_pos_and_yaw_ << msg->position.x, msg->position.y, msg->position.z, msg->yaw;
  }

  void TopicManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_world_.odom_pos(0) = msg->pose.pose.position.x;
    odom_world_.odom_pos(1) = msg->pose.pose.position.y;
    odom_world_.odom_pos(2) = msg->pose.pose.position.z;

    odom_world_.odom_vel(0) = msg->twist.twist.linear.x;
    odom_world_.odom_vel(1) = msg->twist.twist.linear.y;
    odom_world_.odom_vel(2) = msg->twist.twist.linear.z;

    odom_world_.odom_orient.w() = msg->pose.pose.orientation.w;
    odom_world_.odom_orient.x() = msg->pose.pose.orientation.x;
    odom_world_.odom_orient.y() = msg->pose.pose.orientation.y;
    odom_world_.odom_orient.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = odom_world_.odom_orient.toRotationMatrix().block<3, 1>(0, 0);
    odom_world_.odom_yaw = atan2(rot_x(1), rot_x(0));
  }

  bool TopicManager::getNewTargetFlag() const
  {
    return new_target_flag_.load();
  }

  void TopicManager::setNewTargetFlag(const bool &new_target_flag)
  {
    new_target_flag_.store(new_target_flag);
  }

  Eigen::Vector4d TopicManager::getNewTarget() const
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return next_pos_and_yaw_;
  }

  ctrl_manager::OdomWorld TopicManager::getOdom() const
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return odom_world_;
  }

  void TopicManager::publishThrustAndOmega(const Eigen::Vector4d &ctrl_cmd) const
  {
    // RCLCPP_INFO(node_->get_logger(), "Publishing thrust and omega commands");
    std::lock_guard<std::mutex> lock(task_mutex_);
    quadrotor_msgs::msg::ThrustAndOmega msg;
    msg.thrust = ctrl_cmd(0);
    msg.omega_x = ctrl_cmd(1);
    msg.omega_y = ctrl_cmd(2);
    msg.omega_z = ctrl_cmd(3);

    thrust_omega_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "publishThrustAndOmega Published ");
  }

} // namespace ctrl_manager
