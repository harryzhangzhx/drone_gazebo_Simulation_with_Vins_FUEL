#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

// ros::Publisher pub1;
// ros::Publisher pub2;
// ros::Publisher pub3;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
// rclcpp::Publisher<>::SharedPtr pub3;
string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::msg::Odometry odom;
nav_msgs::msg::Path waypoints;

// series waypoint needed
std::deque<nav_msgs::msg::Path> waypointSegments;
rclcpp::Time trigged_time;

void load_seg(const std::shared_ptr<rclcpp::Node>& node, int segid, const rclcpp::Time& time_base) {
  std::string seg_str = boost::str(bfmt("seg%d/") % segid);
  double yaw;
  double time_from_start;
  RCLCPP_INFO(node->get_logger(), "Getting segment %d", segid);

  if (!node->get_parameter(seg_str + "yaw", yaw)) {
    RCLCPP_ERROR(node->get_logger(), "Parameter %syaw not found!", seg_str.c_str());
    throw std::runtime_error("Parameter error");
  }
  if (!(yaw > -3.1499999 && yaw < 3.14999999)) {
    RCLCPP_ERROR(node->get_logger(), "yaw=%.3f is out of bounds", yaw);
    throw std::runtime_error("Invalid yaw");
  }

  
  if (!node->get_parameter(seg_str + "time_from_start", time_from_start)) {
    RCLCPP_ERROR(node->get_logger(), "Parameter %stime_from_start not found!", seg_str.c_str());
    throw std::runtime_error("Parameter error");
    }
  if (time_from_start < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "time_from_start=%.3f is negative", time_from_start);
    throw std::runtime_error("Invalid time_from_start");
  }

  std::vector<double> ptx, pty, ptz;
  if (!node->get_parameter(seg_str + "x", ptx) ||
      !node->get_parameter(seg_str + "y", pty) ||
      !node->get_parameter(seg_str + "z", ptz)) {
    RCLCPP_ERROR(node->get_logger(), "Missing position parameter(s)!");
    throw std::runtime_error("Parameter error");
  }
  if (ptx.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Empty position vector!");
    throw std::runtime_error("Empty vector");
  }
  if (ptx.size() != pty.size() || ptx.size() != ptz.size()) {
    RCLCPP_ERROR(node->get_logger(), "Position vector sizes do not match!");
    throw std::runtime_error("Vector size mismatch");
  }

  nav_msgs::msg::Path path_msg;

  path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

  double baseyaw = tf2::getYaw(odom.pose.pose.orientation);

  for (size_t k = 0; k < ptx.size(); ++k) {
    geometry_msgs::msg::PoseStamped pt;
    //pt.pose.orientation = tf2::createQuaternionMsgFromYaw(baseyaw + yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, baseyaw + yaw);
    pt.pose.orientation = tf2::toMsg(q);    
    Eigen::Vector2d dp(ptx.at(k), pty.at(k));
    Eigen::Vector2d rdp;
    rdp.x() = std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
    rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();
    pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
    pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
    pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
    path_msg.poses.push_back(pt);
  }

  waypointSegments.push_back(path_msg);
}

void load_waypoints(const std::shared_ptr<rclcpp::Node>& node, const rclcpp::Time& time_base) {
int seg_cnt = 0;
waypointSegments.clear();

// Get the segment count parameter
if (!node->get_parameter("segment_cnt", seg_cnt)) {
  RCLCPP_ERROR(node->get_logger(), "Parameter 'segment_cnt' not found!");
  throw std::runtime_error("Parameter error");
}

for (int i = 0; i < seg_cnt; ++i) {
  load_seg(node, i, time_base); // Assuming you update this to use ROS 2 node methods
  if (i > 0) {
    if (!(rclcpp::Time(waypointSegments[i - 1].header.stamp) < rclcpp::Time(waypointSegments[i].header.stamp))) {
      RCLCPP_ERROR(node->get_logger(), "Segment %d stamp is not after previous segment!", i);
      throw std::runtime_error("Segment stamp error");
    }
  }
}

  RCLCPP_INFO(rclcpp::get_logger("waypoint_generator"), "Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
  waypoints.header.frame_id = std::string("world");
  waypoints.header.stamp = rclcpp::Clock().now();
  pub1->publish(waypoints);
  geometry_msgs::msg::PoseStamped init_pose;
  init_pose.header = odom.header;
  init_pose.pose = odom.pose.pose;
  waypoints.poses.insert(waypoints.poses.begin(), init_pose);
  // pub2.publish(waypoints);
  waypoints.poses.clear();
}

void publish_waypoints_vis() {
  nav_msgs::msg::Path wp_vis = waypoints;
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.header.frame_id = std::string("world");
  poseArray.header.stamp = rclcpp::Clock().now();

  {
    geometry_msgs::msg::Pose init_pose;
    init_pose = odom.pose.pose;
    poseArray.poses.push_back(init_pose);
  }

  for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
    geometry_msgs::msg::Pose p;
    p = it->pose;
    poseArray.poses.push_back(p);
  }
  pub2->publish(poseArray);
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, std::shared_ptr<rclcpp::Node> node) {
  is_odom_ready = true;
  odom = *msg;

  if (waypointSegments.size()) {
    rclcpp::Time expected_time = waypointSegments.front().header.stamp;
    if (rclcpp::Time(odom.header.stamp) >= rclcpp::Time(expected_time)) {
      waypoints = waypointSegments.front();

      std::stringstream ss;
      ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
      for (auto& pose_stamped : waypoints.poses) {
        ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") % pose_stamped.pose.position.x %
                pose_stamped.pose.position.y % pose_stamped.pose.position.z %
                pose_stamped.pose.orientation.w % pose_stamped.pose.orientation.x %
                pose_stamped.pose.orientation.y % pose_stamped.pose.orientation.z
           << std::endl;
      }
      RCLCPP_INFO_STREAM(node->get_logger(), ss.str());

      publish_waypoints_vis();
      publish_waypoints();

      waypointSegments.pop_front();
    }
  }
}

void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, std::shared_ptr<rclcpp::Node> node) {
  /*    if (!is_odom_ready) {
          ROS_ERROR("[waypoint_generator] No odom!");
          return;
      }*/

  trigged_time = rclcpp::Clock().now();  // odom.header.stamp;
  // ROS_ASSERT(trigged_time > ros::Time(0));

  //ros::NodeHandle n("~");
  //rclcpp::Node::SharedPtr node;
  
  //n.param("waypoint_type", waypoint_type, string("manual"));
  // Declare with default value (does nothing if already declared)
    // node->declare_parameter<std::string>("waypoint_type", "manual");

    // Get the parameter value
    node->get_parameter("waypoint_type", waypoint_type);


  if (waypoint_type == string("circle")) {
    waypoints = circle();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("eight")) {
    waypoints = eight();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("point")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("series")) {
    load_waypoints(node, trigged_time);
  } else if (waypoint_type == string("manual-lonely-waypoint")) {
    if (msg->pose.position.z > -0.1) {
      // if height > 0, it's a valid goal;
      geometry_msgs::msg::PoseStamped pt = *msg;
      waypoints.poses.clear();
      waypoints.poses.push_back(pt);
      publish_waypoints_vis();
      publish_waypoints();
    } else {
      RCLCPP_WARN(node->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
    }
  } else {
    if (msg->pose.position.z > 0) {
      // if height > 0, it's a normal goal;
      geometry_msgs::msg::PoseStamped pt = *msg;
      if (waypoint_type == string("noyaw")) {
        double yaw = tf2::getYaw(odom.pose.pose.orientation);
        //pt.pose.orientation = tf2::createQuaternionMsgFromYaw(yaw);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pt.pose.orientation = tf2::toMsg(q);  
      }
      waypoints.poses.push_back(pt);
      publish_waypoints_vis();
    } else if (msg->pose.position.z > -1.0) {
      // if 0 > height > -1.0, remove last goal;
      if (waypoints.poses.size() >= 1) {
        waypoints.poses.erase(std::prev(waypoints.poses.end()));
      }
      publish_waypoints_vis();
    } else {
      // if -1.0 > height, end of input
      if (waypoints.poses.size() >= 1) {
        publish_waypoints_vis();
        publish_waypoints();
      }
    }
  }
}

void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, std::shared_ptr<rclcpp::Node> node)  {
  if (!is_odom_ready) {
    RCLCPP_ERROR(node->get_logger(), "[waypoint_generator] No odom!");
    return;
  }

  RCLCPP_WARN(node->get_logger(), "[waypoint_generator] Trigger!");
  trigged_time = odom.header.stamp;
  //RCLCPP_ASSERT(trigged_time > rclcpp::Time(0));
  if (!(trigged_time > rclcpp::Time(0))) {
    RCLCPP_ERROR(node->get_logger(), "trigged_time is not greater than zero.");
    return;
  }


  //ros::NodeHandle n("~");
  //n.param("waypoint_type", waypoint_type, string("manual"));

  RCLCPP_ERROR_STREAM(node->get_logger(), "Pattern " << waypoint_type << " generated!");
  if (waypoint_type == string("free")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("circle")) {
    waypoints = circle();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("eight")) {
    waypoints = eight();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("point")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("series")) {
    load_waypoints(node, trigged_time);
  }
}

int main(int argc, char** argv) {
  //rclcpp::init(argc, argv, "waypoint_generator");
  //ros::NodeHandle n("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_generator");
  //n.param("waypoint_type", waypoint_type, string("manual"));
  // Declare with default value (does nothing if already declared)
    // node->declare_parameter<std::string>("waypoint_type", "manual");

    // Get the parameter value
    node->get_parameter("waypoint_type", waypoint_type);

  //ros::Subscriber sub1 = n->subscribe("odom", 10, odom_callback);
  //ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
  //ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);
  auto sub1 = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, 
    [node](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    odom_callback(msg, node);  // Your custom function
  });
  auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, [node](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    goal_callback(msg, node);  // Your custom function
  });
  auto sub3 = node->create_subscription<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10, [node](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    traj_start_trigger_callback(msg, node);  // Your custom function
  });
  
  //pub1 = n.advertise<nav_msgs::msg::Path>("waypoints", 50);
  //pub2 = n.advertise<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
  pub1 = node->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
  pub2 = node->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
    
  trigged_time = rclcpp::Time(0);

  rclcpp::spin(node);
  return 0;
}