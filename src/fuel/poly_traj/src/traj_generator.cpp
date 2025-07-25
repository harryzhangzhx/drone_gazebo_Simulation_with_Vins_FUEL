#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <swarmtal_msgs/msg/drone_onboard_command.hpp> //idk if this one should be changed
#include </poly_traj/polynomial_traj.h>                //originally included itself?

using namespace std;

// ros::Publisher state_pub, pos_cmd_pub, traj_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker> SharedPtr state_pub;
clcpp::Publisher<visualization_msgs::msg::Marker> SharedPtr traj_pub;
rclcpp::Publisher<swarmtal_msgs::msg::drone_onboard_command> SharedPtr pos_cmd_pub; // we have to see how to migrate custom packages

nav_msgs::msg::Odometry odom;
bool have_odom;

void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id, const rclcpp::Node::SharedPtr &node)
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  // mk.header.stamp = rclcpp::Time::now(); //change to node->now(), and pass node into the function?
  mk.header.stamp = node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;

  traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++)
  {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
  // ros::Duration(0.001).sleep();
  rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color, const rclcpp::Node::SharedPtr &node)
{
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  // mk_state.header.stamp = rclcpp::Time::now(); //add node into the function?
  mk_state.header.stamp = node->now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub->publish(mk_state);
}

void odomCallbck(const nav_msgs::msg::Odometry &msg)
{
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;

  odom = msg;
  have_odom = true;
}

int main(int argc, char **argv)
{
  /* ---------- initialize ---------- */
  // ros::init(argc, argv, "traj_generator");
  // ros::NodeHandle node;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_generator");

  // ros::Subscriber odom_sub = node.subscribe("/uwb_vicon_odom", 50, odomCallbck);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/uwb_vicon_odom", 50, odomCallback);

  // traj_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/traj_vis", 10);
  // state_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/cmd_vis", 10);
  traj_pub = node->create_publisher<visualization_msgs::msg::Marker>("/traj_generator/traj_vis", 10);
  state_pub = node->create_publisher<visualization_msgs::msg::Marker>("/traj_generator/cmd_vis", 10);

  // pos_cmd_pub =
  // node.advertise<quadrotor_msgs::PositionCommand>("/traj_generator/position_cmd",
  // 50);

  // pos_cmd_pub =
  //   node.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 10);
  pos_cmd_pub = node->create_publisher<swarmtal_msgs::msg::drone_onboard_command>("/drone_commander/onboard_command", 10);

  rclcpp::sleep_for(std::chrono::seconds(1));

  /* ---------- wait for odom ready ---------- */
  have_odom = false;
  while (!have_odom && rclcpp::ok())
  {
    cout << "no odomeetry." << endl;
    // ros::Duration(0.5).sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    // ros::spinOnce();
    rclcpp::spin_some(node);
  }

  /* ---------- generate trajectory using close-form minimum jerk ---------- */
  Eigen::MatrixXd pos(9, 3);
  // pos.row(0) =
  //     Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
  //     odom.pose.pose.position.z);

  pos.row(0) =
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  // pos.row(0) = Eigen::Vector3d(-2, 0, 1);
  pos.row(1) = Eigen::Vector3d(-0.5, 0.5, 1);
  pos.row(2) = Eigen::Vector3d(0, 0, 1);
  pos.row(3) = Eigen::Vector3d(0.5, -0.5, 1);
  pos.row(4) = Eigen::Vector3d(1, 0, 1);
  pos.row(5) = Eigen::Vector3d(0.5, 0.5, 1);
  pos.row(6) = Eigen::Vector3d(0, 0, 1);
  pos.row(7) = Eigen::Vector3d(-0.5, -0.5, 1);
  pos.row(8) = Eigen::Vector3d(-1, 0, 1);

  Eigen::VectorXd time(8);
  time(0) = 2.0;
  time(1) = 1.5;
  time(2) = 1.5;
  time(3) = 1.5;
  time(4) = 1.5;
  time(5) = 1.5;
  time(6) = 1.5;
  time(7) = 2.0;

  Eigen::MatrixXd poly = generateTraj(pos, time);
  cout << "poly:\n"
       << poly << endl;

  cout << "pos:\n"
       << pos << endl;

  cout << "pos 0 1 2: " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly(i, j), cy[j] = poly(i, j + 6), cz[j] = poly(i, j + 12);
    }
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }
  poly_traj.init();
  vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();

  displayPathWithColor(traj_vis, 0.05, Eigen::Vector4d(1, 0, 0, 1), 1);

  /* ---------- publish command ---------- */
  rclcpp::Time start_time = node->now();
  rclcpp::Time time_now;

  // ros::Duration(0.1).sleep();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  swarmtal_msgs::msg::drone_onboard_command cmd;
  cmd.command_type = swarmtal_msgs::msg::drone_onboard_command::CTRL_POS_COMMAND;
  cmd.param1 = 0;
  cmd.param2 = 0;
  cmd.param3 = 0;
  cmd.param4 = 666666;
  cmd.param5 = 0;
  cmd.param6 = 0;
  cmd.param7 = 0;
  cmd.param8 = 0;
  cmd.param9 = 0;
  cmd.param10 = 0;

  while (rclcpp::ok())
  {
    time_now = node->now();
    double tn = (time_now - start_time).toSec();
    Eigen::Vector3d pt = poly_traj.evaluate(tn);
    Eigen::Vector3d vel = poly_traj.evaluateVel(tn);
    Eigen::Vector3d acc = poly_traj.evaluateAcc(tn);

    cmd.param1 = int(pt(0) * 10000);
    cmd.param2 = int(pt(1) * 10000);
    cmd.param3 = int(pt(2) * 10000);

    cmd.param5 = int(vel(0) * 10000);
    cmd.param6 = int(vel(1) * 10000);

    cmd.param7 = 0;
    cmd.param8 = int(acc(0) * 10000);
    cmd.param9 = int(acc(1) * 10000);

    pos_cmd_pub->publish(cmd);

    drawState(pt, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
    drawState(pt, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
    // ros::Duration(0.01).sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  // ros::spin();
  rclcpp::spin(node);
  return 0;
}