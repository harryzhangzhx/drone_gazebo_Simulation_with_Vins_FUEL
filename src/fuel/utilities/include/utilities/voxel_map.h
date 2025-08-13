#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <queue>
#include <tuple>
#include <memory>
#include <mutex>

struct VoxelMapParam // maybe different name
{
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map boundaries
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_; // maximum and minimum indices
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;

  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                           // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit
  double max_ray_length_;
  double local_bound_inflate_;
  int local_map_margin_;
  double unknown_flag_;
};

class VoxelMap
{
public:
  enum OCCUPANCY
  {
    UNKNOWN,
    FREE,
    OCCUPIED
  };

  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &idx);

  int toAddress(const int &x, const int &y, const int &z);
  int toAddress(const Eigen::Vector3i &idx);

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx);
  void indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos);

  bool isInBox(const Eigen::Vector3i &idx);
  bool isInBox(const Eigen::Vector3d &pos);

  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);

  int getOccupancy(const Eigen::Vector3i &idx);
  int getOccupancy(const Eigen::Vector3d &pos);

  double getDistance(const Eigen::Vector3d &pos);
  double getDistance(const Eigen::Vector3i &idx);

  double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);

  int getInflateOccupancy(const Eigen::Vector3i &idx);

  int getInflateOccupancy(const Eigen::Vector3d &pos);

public:
  std::mutex mtx_;
  VoxelMapParam map_params_;                     // TODO Oliver, please make this private and use getter/setter for access
  std::vector<double> occupancy_buffer_;         // TODO same as above
  std::vector<int8_t> occupancy_buffer_inflate_; // TODO same as above
  std::vector<double> distance_buffer_;          // TODO same as above
  std::vector<double> distance_buffer_neg_;      // TODO same as above
};
