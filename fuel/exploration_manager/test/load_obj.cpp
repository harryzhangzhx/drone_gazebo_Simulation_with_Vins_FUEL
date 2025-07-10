#include <iostream>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "load_obj");
  // ros::NodeHandle node("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("load_obj");

  // ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/load_obj/cloud", 10);
  auto cloud_pub = node->create_publisher<sensor_msgs::PointCloud2>("/load_obj/cloud, 10");

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // pcl::io::loadOBJFile("/home/boboyu/Downloads/AnyConv.com__truss_bridge.obj", cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/boboyu/Downloads/pp.pcd", cloud);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";

  // Rotate the cloud
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x;
    pr.y = -pt.z;
    pr.z = pt.y;
    cloud.points[i] = pr;
  }

  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(cloud, cloud2);

  while (rclcpp::ok())
  {
    cloud_pub.publish(cloud2);
    // ros::Duration(0.2).sleep();
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "Cloud published!" << std::endl;

  return 1;
}
