#include "rclcpp/rclcpp.hpp"
#include "controller_manager.h"
#include "utilities/common_types.h"
#include "utilities/yaml_loader.h"

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller_manager");
  auto params = utilities::loadParamsFromYaml("utilities/init.yaml");
  ctrl_manager::ControllerManager controller_manager(node, params);

  controller_manager.start();
  RCLCPP_INFO(node->get_logger(), "Controller Manager started");

  rclcpp::spin(node);

  controller_manager.stop();
  RCLCPP_INFO(node->get_logger(), "Controller Manager stopped");

  rclcpp::shutdown();
  return 0;
}
