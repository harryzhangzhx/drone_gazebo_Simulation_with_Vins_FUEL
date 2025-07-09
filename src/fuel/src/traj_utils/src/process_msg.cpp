#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <vector>
#include <cmath>

class CloudProcessor : public rclcpp::Node {
public:
  CloudProcessor() : Node("process_msg") {
    // Set map origin
    map_origin_ << -20.0, -10.0, -1.0;

    // Create publisher and subscriber
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/process_msg/global_cloud", 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_generator/global_cloud", 10,
      std::bind(&CloudProcessor::cloudCallback, this, std::placeholders::_1)
    );

    RCLCPP_WARN(this->get_logger(), "[process_msg]: ready.");
  }

private:
  void inflatePoint(const Eigen::Vector3d& pt, int step, std::vector<Eigen::Vector3d>& pts) {
    const double res = 0.1;
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z)
          pts.emplace_back(pt + Eigen::Vector3d(x, y, z) * res);
  }
  

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto tn = this->now();
    if ((tn - last_cloud_time_).seconds() < 5.0) {
      return;
    }
    last_cloud_time_ = tn;

    pcl::PointCloud<pcl::PointXYZ> pts;
    pcl::PointCloud<pcl::PointXYZ> inflated_pts;
    pcl::fromROSMsg(*msg, pts);

    for (const auto& p : pts.points) {
      Eigen::Vector3d pt(p.x, p.y, p.z);

      // Grid snapping to resolution 0.1
      for (int i = 0; i < 3; ++i)
        pt(i) = std::floor((pt(i) - map_origin_(i)) * 10.0);
      for (int i = 0; i < 3; ++i)
        pt(i) = (pt(i) + 0.5) * 0.1 + map_origin_(i);

      std::vector<Eigen::Vector3d> inflated;
      inflatePoint(pt, 1, inflated);

      for (const auto& pi : inflated)
        inflated_pts.emplace_back(pcl::PointXYZ(pi.x(), pi.y(), pi.z()));
    }

    inflated_pts.width = inflated_pts.size();
    inflated_pts.height = 1;
    inflated_pts.is_dense = true;
    inflated_pts.header.frame_id = "world";

    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(inflated_pts, cloud);
    cloud_pub_->publish(cloud);
  }

  Eigen::Vector3d map_origin_;
  rclcpp::Time last_cloud_time_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudProcessor>());
  rclcpp::shutdown();
  return 0;
}


// original code using global variables
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "visualization_msgs/msg/marker.hpp"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <iostream>
// #include <vector>

// using namespace std;

// Eigen::Vector3d map_origin_;
// rclcpp::Time last_cloud_time_;

// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;


// void inflatePoint(const Eigen::Vector3d& pt, int step, vector<Eigen::Vector3d>& pts) {
//   const double res = 0.1;
//   int num = 0;
//   for (int x = -step; x <= step; ++x)
//     for (int y = -step; y <= step; ++y)
//       for (int z = -step; z <= step; ++z) {
//         pts[num++] = Eigen::Vector3d(pt(0) + x * res, pt(1) + y * res, pt(2) + z * res);
//       }
// }

// void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//   auto tn = rclcpp::Clock().now();
//   if ((tn - last_cloud_time_).seconds() < 5) {
//     return;
//   }
//   last_cloud_time_ = tn;

//   pcl::PointCloud<pcl::PointXYZ> pts;
//   pcl::PointCloud<pcl::PointXYZ> pts2;
//   pcl::fromROSMsg(*msg, pts);
//   vector<Eigen::Vector3d> inf_pts(27);

//   for (int i = 0; i < pts.points.size(); ++i) {
//     Eigen::Vector3d pt;
//     pt(0) = pts[i].x;
//     pt(1) = pts[i].y;
//     pt(2) = pts[i].z;
//     for (int i = 0; i < 3; ++i)
//       pt(i) = floor((pt(i) - map_origin_(i)) * 10);
//     for (int i = 0; i < 3; ++i)
//       pt(i) = (pt(i) + 0.5) * 0.1 + map_origin_(i);
//     inflatePoint(pt, 1, inf_pts);
//     for (auto pi : inf_pts) {
//       pcl::PointXYZ pj;
//       pj.x = pi(0);
//       pj.y = pi(1);
//       pj.z = pi(2);
//       pts2.points.push_back(pj);
//     }
//   }
//   pts2.width = pts2.points.size();
//   pts2.height = 1;
//   pts2.is_dense = true;
//   pts2.header.frame_id = "world";

//   sensor_msgs::msg::PointCloud2 cloud;
//   pcl::toROSMsg(pts2, cloud);
//   cloud_pub_->publish(cloud);
// }

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<rclcpp::Node>("process_msg");

//   auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "/map_generator/global_cloud", 10, cloudCallback);
//   cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
//     "/process_msg/global_cloud", 10);

//   map_origin_ << -20, -10, -1;

//   rclcpp::sleep_for(std::chrono::seconds(1));

//   RCLCPP_WARN(node->get_logger(), "[process_msg]: ready.");

//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }