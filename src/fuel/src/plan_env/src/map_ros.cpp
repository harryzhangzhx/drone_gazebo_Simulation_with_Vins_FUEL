#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>

namespace fast_planner {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  // node_->declare_parameter("map_ros/fx", 387.229248046875);
  // node_->declare_parameter("map_ros/fy", 387.229248046875);
  // node_->declare_parameter("map_ros/cx", 321.04638671875);
  // node_->declare_parameter("map_ros/cy", 243.44969177246094);
  // node_->declare_parameter("map_ros/depth_filter_maxdist", 5.0);
  // node_->declare_parameter("map_ros/depth_filter_mindist", 0.2);
  // node_->declare_parameter("map_ros/depth_filter_margin", 2);
  // node_->declare_parameter("map_ros/k_depth_scaling_factor", 1000.0);
  // node_->declare_parameter("map_ros/skip_pixel", 2);
  // node_->declare_parameter("map_ros/esdf_slice_height", 0.3);
  // node_->declare_parameter("map_ros/visualization_truncate_height", 10.0);
  // node_->declare_parameter("map_ros/visualization_truncate_low", -2.0);
  // node_->declare_parameter("map_ros/show_occ_time", false);
  // node_->declare_parameter("map_ros/show_esdf_time", false);
  // node_->declare_parameter("map_ros/show_all_map", false);
  // node_->declare_parameter("map_ros/frame_id", std::string("world"));
  // double fx_, fy_, cx_, cy_;

  node_->get_parameter("map_ros/fx", fx_);
  node_->get_parameter("map_ros/fy", fy_);
  node_->get_parameter("map_ros/cx", cx_);
  node_->get_parameter("map_ros/cy", cy_);
  node_->get_parameter("map_ros/depth_filter_maxdist", depth_filter_maxdist_);
  node_->get_parameter("map_ros/depth_filter_mindist", depth_filter_mindist_);
  node_->get_parameter("map_ros/depth_filter_margin", depth_filter_margin_);
  node_->get_parameter("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_);
  node_->get_parameter("map_ros/skip_pixel", skip_pixel_);

  node_->get_parameter("map_ros/esdf_slice_height", esdf_slice_height_);
  node_->get_parameter("map_ros/visualization_truncate_height", visualization_truncate_height_);
  node_->get_parameter("map_ros/visualization_truncate_low", visualization_truncate_low_);
  node_->get_parameter("map_ros/show_occ_time", show_occ_time_);
  node_->get_parameter("map_ros/show_esdf_time", show_esdf_time_);
  node_->get_parameter("map_ros/show_all_map", show_all_map_);
  node_->get_parameter("map_ros/frame_id", frame_id_);


  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = std::normal_distribution<double>(0, 0.1);
  std::random_device rd;
  eng_ = std::default_random_engine(rd());

  using namespace std::chrono_literals;
  esdf_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MapROS::updateESDFCallback, this));
  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MapROS::visCallback, this));

  map_all_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", 10);

  map_start_time_ = node_->now();
}

void MapROS::visCallback() {
  publishMapLocal();
  if (show_all_map_) {
    static double tpass = 0.0;
    tpass += 0.05; // Approximate timer period
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;
    }
  }
}

void MapROS::updateESDFCallback() {
  if (!esdf_need_update_) return;
  auto t1 = node_->now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = node_->now();
  esdf_time_ += (t2 - t1).seconds();
  max_esdf_time_ = std::max(max_esdf_time_, (t2 - t1).seconds());
  esdf_num_++;
  if (show_esdf_time_)
    RCLCPP_WARN(node_->get_logger(), "ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), esdf_time_ / esdf_num_,
             max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) {
  camera_pos_(0) = pose->pose.pose.position.x;
  camera_pos_(1) = pose->pose.pose.position.y;
  camera_pos_(2) = pose->pose.pose.position.z;
  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  camera_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                                 pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = node_->now();

  // generate point cloud, update map
  proessDepthImage();
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = node_->now();
  fuse_time_ += (t2 - t1).seconds();
  max_fuse_time_ = std::max(max_fuse_time_, (t2 - t1).seconds());
  fuse_num_ += 1;
  if (show_occ_time_)
    RCLCPP_WARN(node_->get_logger(), "Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}

void MapROS::cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) {
  camera_pos_(0) = pose->pose.pose.position.x;
  camera_pos_(1) = pose->pose.pose.position.y;
  camera_pos_(2) = pose->pose.pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                                 pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = depth;
      pt_cur(1) = -(u - cx_) * depth / fx_;
      pt_cur(2) = -(v - cy_) * depth / fy_;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_->publish(cloud_msg);

  // Output time and known volume
  double time_now = (node_->now() - map_start_time_).seconds();
  double known_volumn = 0;

  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;
      }

  std::ofstream file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/curve1.txt",
                std::ios::app);
  file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_->publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_->publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_->publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_->publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_->publish(cloud_msg);
}
}  // namespace fast_planner