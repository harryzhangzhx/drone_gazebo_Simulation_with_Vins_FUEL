#include "controller_manager/service_manager.h"
#include <chrono>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

namespace ctrl_manager
{

    ServiceManager::ServiceManager(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
    }

    const std::vector<double> &ServiceManager::getRawMap() const
    {
        return raw_map_;
    }

    const std::vector<double> &ServiceManager::getInflatedMap() const
    {
        return inflated_map_;
    }

} // namespace ctrl_manager
