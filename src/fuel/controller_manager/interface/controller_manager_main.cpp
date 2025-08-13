#include <sys/stat.h>
#include <filesystem>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "controller_manager.h"
#include "utilities/common_types.h"
#include "utilities/yaml_loader.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller_manager_node");
  RCLCPP_INFO(node->get_logger(), "controller_manager created");

  // Check the path
  std::string base = __FILE__;
  auto pos = base.find_last_of("/\\");
  base = (pos == std::string::npos) ? "." : base.substr(0, pos);
  std::string yaml_path = base + "/../../utilities/include/utilities/init.yaml";
  utilities::AllParams params = utilities::loadParamsFromYaml(yaml_path);
  RCLCPP_INFO(node->get_logger(), "YAML Loaded");

  ctrl_manager::ControllerManager controller_manager(node, params);
  RCLCPP_INFO(node->get_logger(), "controller_manager instantiated");

  controller_manager.start();
  RCLCPP_INFO(node->get_logger(), "Controller Manager started");

  rclcpp::spin(node);

  controller_manager.stop();
  RCLCPP_INFO(node->get_logger(), "Controller Manager stopped");

  rclcpp::shutdown();
  return 0;
}
