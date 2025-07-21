#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <cmath>

#include "stdint.h"

#include "chrono"
#include "iostream"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DroneCircleNode : public rclcpp::Node
{
public:
  DroneCircleNode() : Node("drone_circle_node")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void
    {
      if (offboard_setpoint_counter_ == 10)
      {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
      }

      publish_offboard_control_mode();
      publish_trajectory_setpoint();
      if (offboard_setpoint_counter_ < 11)
      {
        offboard_setpoint_counter_++;
      }
    };

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

  void arm();
  void disarm();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

  std::atomic<uint64_t> timestamp_;
  float yaw_angle_ = 0.0f;

  float radius_ = 5.0f;
  uint64_t offboard_setpoint_counter_;

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void DroneCircleNode::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command sent yahoo");
}

void DroneCircleNode::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void DroneCircleNode::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

// // For circling motion
// void DroneCircleNode::publish_trajectory_setpoint()
// {
// 	TrajectorySetpoint msg{};
// 	msg.position = {0.0, 0.0, -5.0};

//   msg.position = {radius_*std::cos(yaw_angle_), radius_*std::sin(yaw_angle_), -5.0};
//   //msg.position.x = radius_*std::cos(yaw_angle_);
//   //msg.position.y = radius_*std::sin(yaw_angle_);

// 	msg.yaw = yaw_angle_; // [-PI:PI]
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	trajectory_setpoint_publisher_->publish(msg);
//   yaw_angle_ += 0.1f; //adjust the yaw angle to fly in a circle

//   if (yaw_angle_ > M_PI) {
//       yaw_angle_ -= 2.0f*M_PI;
//   }
// }

// For hovering motion
void DroneCircleNode::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = {0.0, 0.0, -1.0}; // Hover at (0, 0, -1)
  msg.yaw = 0.0f;                  // Keep yaw fixed
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void DroneCircleNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{

  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char **argv)
{

  std::cout << "Starting Circular Offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneCircleNode>());

  rclcpp::shutdown();
  return 0;
}
