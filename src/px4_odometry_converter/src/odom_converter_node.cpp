#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <array>

using std::placeholders::_1;

class OdometryConverter : public rclcpp::Node
{
public:
    OdometryConverter()
        : Node("px4_odometry_converter")
    {
        // Subscriber to PX4 odometry
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&OdometryConverter::odometry_callback, this, _1));

        // Publisher for standard Odometry
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
        // Publisher for rotated Odometry
        rotated_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_rotated", 100);
        // Publisher for rotated PoseStamped
        rotated_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_rotated", 100);

        RCLCPP_INFO(this->get_logger(), "PX4 VehicleOdometry → Odometry converter started.");
    }

private:
    // Helper function for quaternion multiplication (w, x, y, z order)
    std::array<double, 4> quat_multiply(const std::array<double, 4> &q1, const std::array<double, 4> &q2)
    {
        return {
            q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3], // w
            q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2], // x
            q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1], // y
            q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]  // z
        };
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Original odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = msg->position[0];
        odom_msg.pose.pose.position.y = msg->position[1];
        odom_msg.pose.pose.position.z = msg->position[2];
        odom_msg.pose.pose.orientation.w = msg->q[0];
        odom_msg.pose.pose.orientation.x = msg->q[1];
        odom_msg.pose.pose.orientation.y = msg->q[2];
        odom_msg.pose.pose.orientation.z = msg->q[3];
        odom_msg.twist.twist.linear.x = msg->velocity[0];
        odom_msg.twist.twist.linear.y = msg->velocity[1];
        odom_msg.twist.twist.linear.z = msg->velocity[2];
        odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
        odom_msg.twist.twist.angular.y = msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = msg->angular_velocity[2];
        publisher_->publish(odom_msg);

        // Rotated odometry
        auto rotated_odom_msg = odom_msg;
        rotated_odom_msg.header.frame_id = "odom_rotated";
        std::array<double, 4> q_rot = {0.5, -0.5, 0.5, -0.5};
        std::array<double, 4> q_in = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        auto q_out = quat_multiply(q_rot, q_in);
        rotated_odom_msg.pose.pose.orientation.w = q_out[0];
        rotated_odom_msg.pose.pose.orientation.x = q_out[1];
        rotated_odom_msg.pose.pose.orientation.y = q_out[2];
        rotated_odom_msg.pose.pose.orientation.z = q_out[3];
        rotated_publisher_->publish(rotated_odom_msg);

        // Rotated PoseStamped
        geometry_msgs::msg::PoseStamped rotated_pose_msg;
        rotated_pose_msg.header.stamp = this->get_clock()->now();
        rotated_pose_msg.header.frame_id = "odom_rotated";
        rotated_pose_msg.pose = rotated_odom_msg.pose.pose;
        rotated_pose_publisher_->publish(rotated_pose_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotated_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rotated_pose_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryConverter>());
    rclcpp::shutdown();
    return 0;
}