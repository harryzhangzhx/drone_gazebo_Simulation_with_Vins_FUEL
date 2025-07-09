#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <random>
#include <chrono>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

namespace fast_planner {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();

private:
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void updateESDFCallback();
  void visCallback();

  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishDepth();

  void proessDepthImage();

  SDFMap* map_;
  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                          nav_msgs::msg::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  rclcpp::Node::SharedPtr node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> pose_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_, depth_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_;
  rclcpp::TimerBase::SharedPtr esdf_timer_, vis_timer_;

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  std::string frame_id_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  std::unique_ptr<cv::Mat> depth_image_;
  std::vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  rclcpp::Time map_start_time_;

  friend SDFMap;
};
}

#endif