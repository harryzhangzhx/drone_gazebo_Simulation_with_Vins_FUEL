#include "controller_manager/topic_manager.h"
#include <iostream>
#include <functional>

namespace ctrl_manager
{

  TopicManager::TopicManager(std::shared_ptr<rclcpp::Node> node)
      : node_(node)
  {
    next_pos_and_yaw_sub_ = node_->create_subscription<quadrotor_msgs::msg::NextPosAndYaw>(
        "/quadrotor_msgs/next_pos_and_yaw",
        rclcpp::QoS(10),
        std::bind(&TopicManager::nextPosAndYawCallback, this, std::placeholders::_1));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/state_ukf/odom",
        rclcpp::QoS(10),
        std::bind(&TopicManager::odometryCallback, this, std::placeholders::_1));
  }

  void TopicManager::nextPosAndYawCallback(const quadrotor_msgs::msg::NextPosAndYaw::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "nextPosAndYawCallback received message:");
    RCLCPP_INFO(node_->get_logger(), "Position: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);
    RCLCPP_INFO(node_->get_logger(), "Yaw: %f", msg->yaw);

    new_target_flag_ = true;
    next_pos_and_yaw_ << msg->position.x, msg->position.y, msg->position.z, msg->yaw;
  }

  void TopicManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_world_.odom_pos_(0) = msg->pose.pose.position.x;
    odom_world_.odom_pos_(1) = msg->pose.pose.position.y;
    odom_world_.odom_pos_(2) = msg->pose.pose.position.z;

    odom_world_.odom_vel_(0) = msg->twist.twist.linear.x;
    odom_world_.odom_vel_(1) = msg->twist.twist.linear.y;
    odom_world_.odom_vel_(2) = msg->twist.twist.linear.z;

    odom_world_.odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_world_.odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_world_.odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_world_.odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = odom_world_.odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    odom_world_.odom_yaw_ = atan2(rot_x(1), rot_x(0));
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

} // namespace ctrl_manager
