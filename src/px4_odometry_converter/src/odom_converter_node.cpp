#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <array>
#include <px4_ros_com/frame_transforms.h>
#include <Eigen/Geometry>

using std::placeholders::_1;

class OdometryConverter : public rclcpp::Node
{
public:
    OdometryConverter()
    : Node("px4_odometry_converter")
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Subscriber to PX4 odometry
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&OdometryConverter::odometry_callback, this, _1));

        // Publisher for standard Odometry
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 200);
        // Publisher for rotated Odometry
        rotated_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_rotated", 200);
        // Publisher for rotated PoseStamped
        rotated_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_rotated", 200);

        RCLCPP_INFO(this->get_logger(), "PX4 VehicleOdometry → Odometry converter started.");
    }

private:
    // Helper function for quaternion multiplication (w, x, y, z order)
    std::array<double, 4> quat_multiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
        return {
            q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3], // w
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2], // x
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1], // y
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]  // z
        };
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Convert PX4 position (NED) to ROS (ENU)
        Eigen::Vector3d px4_position(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d ros_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_position);

        // Convert PX4 velocity (NED) to ROS (ENU)
        Eigen::Vector3d px4_velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d ros_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(px4_velocity);

        // Convert PX4 orientation (quaternion, wxyz) to ROS (Eigen::Quaterniond)
        std::array<double, 4> px4_q = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        Eigen::Quaterniond px4_quat(px4_q[0], px4_q[1], px4_q[2], px4_q[3]);
        Eigen::Quaterniond ros_quat = px4_ros_com::frame_transforms::px4_to_ros_orientation(px4_quat);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now(); // Use simulation time from /clock
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = ros_position.x();
        odom_msg.pose.pose.position.y = ros_position.y();
        odom_msg.pose.pose.position.z = ros_position.z();
        odom_msg.pose.pose.orientation.w = ros_quat.w();
        odom_msg.pose.pose.orientation.x = ros_quat.x();
        odom_msg.pose.pose.orientation.y = ros_quat.y();
        odom_msg.pose.pose.orientation.z = ros_quat.z();
        odom_msg.twist.twist.linear.x = ros_velocity.x();
        odom_msg.twist.twist.linear.y = ros_velocity.y();
        odom_msg.twist.twist.linear.z = ros_velocity.z();
        odom_msg.twist.twist.angular.x = msg->angular_velocity[0]; // If needed, convert angular velocity too
        odom_msg.twist.twist.angular.y = msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = msg->angular_velocity[2];
        publisher_->publish(odom_msg);

        // Rotated odometry and pose (if needed, apply your rotation logic here)
        auto rotated_odom_msg = odom_msg;
        rotated_odom_msg.header.stamp = this->now();
        rotated_odom_msg.header.frame_id = "odom_rotated";
        std::array<double, 4> q_rot = {0.5, -0.5, 0.5, -0.5};
        std::array<double, 4> q_in = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        auto q_out = quat_multiply(q_rot, q_in);
        rotated_odom_msg.pose.pose.orientation.w = q_out[0];
        rotated_odom_msg.pose.pose.orientation.x = q_out[1];
        rotated_odom_msg.pose.pose.orientation.y = q_out[2];
        rotated_odom_msg.pose.pose.orientation.z = q_out[3];
        rotated_publisher_->publish(rotated_odom_msg);

        geometry_msgs::msg::PoseStamped rotated_pose_msg;
        rotated_pose_msg.header.stamp = this->now();
        rotated_pose_msg.header.frame_id = "odom_rotated";
        rotated_pose_msg.pose = rotated_odom_msg.pose.pose;
        rotated_pose_publisher_->publish(rotated_pose_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotated_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rotated_pose_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryConverter>());
    rclcpp::shutdown();
    return 0;
}