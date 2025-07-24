#pragma once

#include <atomic>
#include <Eigen/Core>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>

#include "controller_manager_types.h"
// #include "messages/srv/get_raw_map.hpp"
// #include "messages/srv/get_inflated_map.hpp"

namespace ctrl_manager
{

  class ServiceManager
  {
  public:
    ServiceManager(std::shared_ptr<rclcpp::Node> node);

    const std::vector<double> &getRawMap() const;
    const std::vector<double> &getInflatedMap() const;

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<double> raw_map_;
    std::vector<double> inflated_map_;
  };

}
