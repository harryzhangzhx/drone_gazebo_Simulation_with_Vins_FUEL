#pragma once

#include <atomic>
#include <Eigen/Core>
#include <mutex>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <utilities/voxel_map.h>
#include "controller_manager_types.h"

#include "quadrotor_msgs/srv/get_map_data.hpp"
#include "quadrotor_msgs/srv/get_map_param.hpp"

namespace ctrl_manager
{

  class ServiceManager
  {
  public:
    ServiceManager(std::shared_ptr<rclcpp::Node> node);

    bool updateMapParam(std::shared_ptr<VoxelMap> voxel_map_ptr = nullptr);
    bool updateMapData(std::shared_ptr<VoxelMap> voxel_map_ptr = nullptr);
    // void updateMapData(std::shared_ptr<VoxelMap> voxel_map_ptr = nullptr);
    bool hasNewMap(std::uint64_t since_version) const;
    std::uint64_t currentVersion() const;

  private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<int8_t> inflated_map_;
    rclcpp::Client<quadrotor_msgs::srv::GetMapData>::SharedPtr map_data_client_;
    rclcpp::Client<quadrotor_msgs::srv::GetMapParam>::SharedPtr map_param_client_;

    // Cached copy + mutex
    std::mutex cache_mtx_;
    std::vector<double> cache_raw_;
    std::vector<int8_t> cache_inflate_;
    std::vector<double> cache_distance_;
    std::vector<double> cache_distance_neg_;
    bool cache_valid_ = false;

    // void onMapDataResponse(rclcpp::Client<quadrotor_msgs::srv::GetMapData>::SharedFuture future,
    //                        std::shared_ptr<VoxelMap> voxel_map_ptr);

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::atomic<bool> request_in_flight_{false}; // Prevent overlapping requests
    std::uint64_t map_version_{0};               // Versioning for freshness checks
    mutable std::mutex version_mtx_;
  };

}
