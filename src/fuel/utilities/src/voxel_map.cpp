#include "utilities/voxel_map.h"

// TODO move functions in here used by the local planners, and the functions those functions use
// TODO document all of the functions and variables

// check if pos is within map_min_boundary and map_max_boundary
bool VoxelMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < map_params_.map_min_boundary_(0) + 1e-4 ||
      pos(1) < map_params_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < map_params_.map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > map_params_.map_max_boundary_(0) - 1e-4 ||
      pos(1) > map_params_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > map_params_.map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

// check if idx is nonnegative and within map_voxel_num
bool VoxelMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    return false;
  if (idx(0) > map_params_.map_voxel_num_(0) - 1 ||
      idx(1) > map_params_.map_voxel_num_(1) - 1 ||
      idx(2) > map_params_.map_voxel_num_(2) - 1)
    return false;
  return true;
}

bool VoxelMap::isInBox(const Eigen::Vector3i &idx)
{
  for (int i = 0; i < 3; i++)
    if (idx(i) < map_params_.box_min_(i) || idx(i) >= map_params_.box_max_(i))
      return false;
  return true;
}

bool VoxelMap::isInBox(const Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    if (pos[i] <= map_params_.box_mind_[i] || pos[i] >= map_params_.box_maxd_[i])
      return false;
  return true;
}

int VoxelMap::toAddress(const int &x, const int &y, const int &z)
{
  return x * map_params_.map_voxel_num_(1) * map_params_.map_voxel_num_(2) + y * map_params_.map_voxel_num_(2) + z;
}

int VoxelMap::toAddress(const Eigen::Vector3i &idx)
{
  return toAddress(idx[0], idx[1], idx[2]);
}

void VoxelMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - map_params_.map_origin_(i)) * map_params_.resolution_inv_);
}

void VoxelMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * map_params_.resolution_ + map_params_.map_origin_(i);
}

void VoxelMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = map_params_.map_origin_, size = map_params_.map_size_;
}

int VoxelMap::getOccupancy(const Eigen::Vector3i &id)
{
  std::lock_guard<std::mutex> g(mtx_);
  if (!isInMap(id))
    return -1;
  double occ = occupancy_buffer_[toAddress(id)];
  if (occ < map_params_.clamp_min_log_ - 1e-3)
    return UNKNOWN;
  if (occ > map_params_.min_occupancy_log_)
    return OCCUPIED;
  return FREE;
}

int VoxelMap::getOccupancy(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

double VoxelMap::getDistance(const Eigen::Vector3i &id)
{
  std::lock_guard<std::mutex> g(mtx_);
  if (!isInMap(id))
    return -1;
  return distance_buffer_[toAddress(id)];
}

double VoxelMap::getDistance(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance(id);
}

double VoxelMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad)
{
  if (!isInMap(pos))
  {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * map_params_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * map_params_.resolution_inv_;

  double values[2][2][2];
  {
    std::lock_guard<std::mutex> g(mtx_);

    for (int x = 0; x < 2; ++x)
      for (int y = 0; y < 2; ++y)
        for (int z = 0; z < 2; ++z)
        {
          Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
          if (!isInMap(current_idx))
          {
            values[x][y][z] = -1.0;
            continue;
          }
          values[x][y][z] = distance_buffer_[toAddress(current_idx)];
        }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * map_params_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * map_params_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= map_params_.resolution_inv_;

  return dist;
}

int VoxelMap::getInflateOccupancy(const Eigen::Vector3i &id)
{
  std::lock_guard<std::mutex> g(mtx_);
  if (!isInMap(id))
    return -1;
  return int(occupancy_buffer_inflate_[toAddress(id)]);
}

int VoxelMap::getInflateOccupancy(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}
