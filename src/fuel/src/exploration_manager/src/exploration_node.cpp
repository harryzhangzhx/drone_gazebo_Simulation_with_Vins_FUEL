// #include <rclcpp/rclcpp.hpp>
// #include <exploration_manager/fast_exploration_fsm.h>

// #include <plan_manage/backward.hpp>
// namespace backward {
// backward::SignalHandling sh;
// }

// using namespace fast_planner;

// int main(int argc, char** argv) {
//   //ros::init(argc, argv, "exploration_node");
//   //ros::NodeHandle nh("~");
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions options;
//   options.allow_undeclared_parameters(true);
//   options.automatically_declare_parameters_from_overrides(true);
//   auto node = rclcpp::Node::make_shared("exploration_node", options);

//   FastExplorationFSM expl_fsm;
//   expl_fsm.init(node);

//   //ros::Duration(1.0).sleep();
//   rclcpp::sleep_for(std::chrono::seconds(1));
//   //ros::spin();
//   rclcpp::spin(node);

//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <exploration_manager/fast_exploration_fsm.h>
#include <plan_manage/backward.hpp>

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("exploration_node", options);

  // FastExplorationFSM expl_fsm;
  // expl_fsm.init(node);  // Set parameters and state (no timers yet)

  auto fsm = std::make_shared<fast_planner::FastExplorationFSM>();
  fsm->init(node);

  rclcpp::sleep_for(std::chrono::seconds(2));  // Optional buffer delay

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}