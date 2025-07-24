// #include <astar2.h>
// #include <sstream>
// #include <plan_env/sdf_map.h>

// using namespace std;
// using namespace Eigen;

// namespace path_planner {
// Astar::Astar() {
// }

// Astar::~Astar() {
//   for (int i = 0; i < allocate_num_; i++)
//     delete path_node_pool_[i];
// }

// void Astar::init(ros::NodeHandle& nh, const std::vector<double>& occupancy_buffer_,
//   const std::vector<char>& occupancy_buffer_inflate_) {

//   tie_breaker_ = 1.0 + 1.0 / 1000;

//   // Astar params
//   nh.param("astar/resolution_astar", resolution_, 0.2);
//   nh.param("astar/lambda_heu", lambda_heu_, 10000.0);
//   nh.param("astar/max_search_time", max_search_time_, 1.0);
//   nh.param("astar/allocate_num", allocate_num_, 1000000);

//   std::cout << "Astar resolution_: " << resolution_ << std::endl;
//   std::cout << "lambda_heu_: " << lambda_heu_ << std::endl;
//   std::cout << "max_search_time_: " << max_search_time_ << std::endl;
//   std::cout << "allocate_num_: " << allocate_num_ << std::endl;

//   this->inv_resolution_ = 1.0 / resolution_;

//   // Map params
//   double x_size, y_size, z_size, map_ground_height, map_resolution;
//   nh.param("sdf_map/map_size_x", x_size, 50.0);
//   nh.param("sdf_map/map_size_y", y_size, 50.0);
//   nh.param("sdf_map/map_size_z", z_size, 10.0);
//   nh.param("sdf_map/ground_height", map_ground_height, -1.0);
//   nh.param("sdf_map/resolution", map_resolution, 0.1);

//   std::cout << "map_size_x: " << x_size << std::endl;
//   std::cout << "map_size_y: " << y_size << std::endl;
//   std::cout << "map_size_z: " << z_size << std::endl;
//   std::cout << "ground_height: " << map_ground_height << std::endl;
//   std::cout << "resolution_: " << map_resolution << std::endl;

//   double p_min, p_occ;
//   nh.param("sdf_map/p_min", p_min, 0.12);
//   nh.param("sdf_map/p_occ", p_occ, 0.80);
//   clamp_min_log_ = logit(p_min);
//   min_occupancy_log_ = logit(p_occ);

//   origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, map_ground_height);
//   map_size_3d_ = Eigen::Vector3d(x_size, y_size, z_size);
//   cout << "origin_: " << origin_.transpose() << endl;
//   cout << "map size: " << map_size_3d_.transpose() << endl;

//   // Try retriving bounding box of map, set box to map size if not specified
//   map_min_boundary_ = origin_;
//   map_max_boundary_ = origin_ + map_size_3d_;
//   Eigen::Vector3d box_mind, box_maxd;
//   vector<string> axis = { "x", "y", "z" };
//   for (int i = 0; i < 3; ++i) {
//     nh.param("sdf_map/box_min_" + axis[i], box_mind[i], map_min_boundary_[i]);
//     nh.param("sdf_map/box_max_" + axis[i], box_maxd[i], map_max_boundary_[i]);
//   }
//   posToIndex(box_mind, box_min_);
//   posToIndex(box_maxd, box_max_);

//   for (int i = 0; i < 3; ++i)
//     map_voxel_num_(i) = ceil(map_size_3d_(i) / map_resolution);

//   path_node_pool_.resize(allocate_num_);
//   cout << "allocate_num_" << allocate_num_ << endl;
//   for (int i = 0; i < allocate_num_; i++) {
//     path_node_pool_[i] = new Node;
//   }

//   std::cout << "size of path_node_pool_: " << path_node_pool_.size() << std::endl;

//   use_node_num_ = 0;
//   iter_num_ = 0;
//   early_terminate_cost_ = 0.0;
//   cout << "path_planner Astar2 init completed" << endl;
// }

// void Astar::setResolution(const double& res) {
//   resolution_ = res;
//   this->inv_resolution_ = 1.0 / resolution_;
// }

// int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {

//   reset();

//   std::cout << "path_planner astar started" << std::endl;

//   std::cout << "size of path_node_pool_: " << path_node_pool_.size() << std::endl;

//   std::cout << "path_planner start_pt: " << start_pt << std::endl;
//   std::cout << "path_planner end_pt: " << end_pt << std::endl;

//   NodePtr cur_node = path_node_pool_[0];
//   cur_node->parent = NULL;
//   cur_node->position = start_pt;

//   std::cout << "cur_node set" << std::endl;

//   posToIndex(start_pt, cur_node->index);
//   cur_node->g_score = 0.0;
//   cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

//   std::cout << "path_planner 1" << std::endl;

//   Eigen::Vector3i end_index;
//   posToIndex(end_pt, end_index);

//   std::cout << "path_planner 2" << std::endl;
//   open_set_.push(cur_node);
//   open_set_map_.insert(make_pair(cur_node->index, cur_node));
//   use_node_num_ += 1;

//   const auto t1 = ros::Time::now();

//   std::cout << "t1: " << t1 << std::endl;

//   /* ---------- search loop ---------- */
//   while (!open_set_.empty()) {
//     std::cout << "size of open_set_: " << open_set_.size() << std::endl;
//     cur_node = open_set_.top();
//     bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
//         abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
//     if (reach_end) {
//       backtrack(cur_node, end_pt);
//       std::cout << "return Reached" << std::endl;
//       return path_planner::REACH_END;
//     }

//     std::cout << "path_planner 5" << std::endl;
//     // Early termination if time up

//     const auto t2 = ros::Time::now();
//     std::cout << "t2: " << t2 << std::endl;
//     if ((t2 - t1).toSec() > max_search_time_) {
//       std::cout << "early return" << std::endl;
//       early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
//       return path_planner::NO_PATH;
//     }

//     open_set_.pop();
//     open_set_map_.erase(cur_node->index);
//     close_set_map_.insert(make_pair(cur_node->index, 1));
//     iter_num_ += 1;

//     Eigen::Vector3d cur_pos = cur_node->position;
//     std::cout << "cur_pos: " << cur_pos << std::endl;
//     Eigen::Vector3d nbr_pos;
//     Eigen::Vector3d step;

//     for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
//       for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
//         for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
//           // std::cout << "dx: " << dx << "\ndy: " << dy << "\ndz: " << dz << std::endl;
//           step << dx, dy, dz;
//           if (step.norm() < 1e-3) continue;
//           nbr_pos = cur_pos + step;
//           // std::cout << "nbr_pos: " << nbr_pos << std::endl;
//           // Check safety
//           if (!isInBox(nbr_pos)) {
//             std::cout << "continue called 0 " << std::endl;
//             continue;
//           }
//           auto inflateOccu = getInflateOccupancy(nbr_pos);
//           std::cout << "inflateOccu: " << inflateOccu << std::endl;
//           auto occu = getOccupancy(nbr_pos);
//           std::cout << "occu: " << occu << std::endl;
//           if ( inflateOccu== 1 || occu == fast_planner::SDFMap::UNKNOWN){
//                 std::cout << "continue called 1 " << std::endl;
//                 continue;
//               }

//           bool safe = true;
//           Vector3d dir = nbr_pos - cur_pos;
//           double len = dir.norm();
//           dir.normalize();
//           for (double l = 0.1; l < len; l += 0.1) {
//             Vector3d ckpt = cur_pos + l * dir;
//             if (getInflateOccupancy(ckpt) == 1 || getOccupancy(ckpt) == fast_planner::SDFMap::UNKNOWN) {
//               safe = false;
//               std::cout << "break called 1 " << std::endl;
//               break;
//             }
//           }
//           if (!safe) {
//             std::cout << "continue called 2 " << std::endl;
//             continue;
//           }

//           // Check not in close set
//           Eigen::Vector3i nbr_idx;
//           posToIndex(nbr_pos, nbr_idx);
//           if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
//             std::cout << "continue called 3 " << std::endl;
//             continue;
//           }

//           NodePtr neighbor;
//           double tmp_g_score = step.norm() + cur_node->g_score;
//           auto node_iter = open_set_map_.find(nbr_idx);
//           if (node_iter == open_set_map_.end()) {
//             neighbor = path_node_pool_[use_node_num_];
//             use_node_num_ += 1;
//             if (use_node_num_ == allocate_num_) {
//               cout << "run out of node pool." << endl;
//               std::cout << "return NO_Path 2" << std::endl;
//               return path_planner::NO_PATH;
//             }
//             neighbor->index = nbr_idx;
//             neighbor->position = nbr_pos;
//           } else if (tmp_g_score < node_iter->second->g_score) {
//             neighbor = node_iter->second;
//           } else {
//             std::cout << "continue called 4 " << std::endl;
//             continue;
//           }

//           neighbor->parent = cur_node;
//           neighbor->g_score = tmp_g_score;
//           neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
//           open_set_.push(neighbor);
//           std::cout << "open_set_ pushed: " << std::endl;
//           open_set_map_[nbr_idx] = neighbor;
//         }
//   }
//   cout << "open set empty, no path!" << endl;
//   // cout << "use node num: " << use_node_num_ << endl;
//   // cout << "iter num: " << iter_num_ << endl;
//   return path_planner::NO_PATH;
// }

// double Astar::getEarlyTerminateCost() {
//   return early_terminate_cost_;
// }

// void Astar::reset() {
//   open_set_map_.clear();
//   close_set_map_.clear();
//   path_nodes_.clear();

//   std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
//   open_set_.swap(empty_queue);
//   for (int i = 0; i < use_node_num_; i++) {
//     path_node_pool_[i]->parent = NULL;
//   }
//   use_node_num_ = 0;
//   iter_num_ = 0;
// }

// double Astar::pathLength(const vector<Eigen::Vector3d>& path) {
//   double length = 0.0;
//   if (path.size() < 2) return length;
//   for (int i = 0; i < path.size() - 1; ++i)
//     length += (path[i + 1] - path[i]).norm();
//   return length;
// }

// void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
//   path_nodes_.push_back(end);
//   path_nodes_.push_back(end_node->position);
//   NodePtr cur_node = end_node;
//   while (cur_node->parent != NULL) {
//     cur_node = cur_node->parent;
//     path_nodes_.push_back(cur_node->position);
//   }
//   reverse(path_nodes_.begin(), path_nodes_.end());
// }

// std::vector<Eigen::Vector3d> Astar::getPath() {
//   return path_nodes_;
// }

// double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
//   double dx = fabs(x1(0) - x2(0));
//   double dy = fabs(x1(1) - x2(1));
//   double dz = fabs(x1(2) - x2(2));
//   double h;
//   double diag = min(min(dx, dy), dz);
//   dx -= diag;
//   dy -= diag;
//   dz -= diag;

//   if (dx < 1e-4) {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
//   }
//   if (dy < 1e-4) {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
//   }
//   if (dz < 1e-4) {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
//   }
//   return tie_breaker_ * h;
// }

// double Astar::getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
//   double dx = fabs(x1(0) - x2(0));
//   double dy = fabs(x1(1) - x2(1));
//   double dz = fabs(x1(2) - x2(2));
//   return tie_breaker_ * (dx + dy + dz);
// }

// double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
//   return tie_breaker_ * (x2 - x1).norm();
// }

// std::vector<Eigen::Vector3d> Astar::getVisited() {
//   vector<Eigen::Vector3d> visited;
//   for (int i = 0; i < use_node_num_; ++i)
//     visited.push_back(path_node_pool_[i]->position);
//   return visited;
// }

// void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
//   idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
// }

// bool Astar::isInBox(const Eigen::Vector3i& id) {
//   for (int i = 0; i < 3; ++i) {
//     if (id[i] < box_min_[i] || id[i] >= box_max_[i]) {
//       return false;
//     }
//   }
//   return true;
// }

// bool Astar::isInMap(const Eigen::Vector3d& pos) {
//   if (pos(0) < map_min_boundary_(0) + 1e-4 || pos(1) < map_min_boundary_(1) + 1e-4 ||
//       pos(2) < map_min_boundary_(2) + 1e-4)
//     return false;
//   if (pos(0) > map_max_boundary_(0) - 1e-4 || pos(1) > map_max_boundary_(1) - 1e-4 ||
//       pos(2) > map_max_boundary_(2) - 1e-4)
//     return false;
//   return true;
// }

// int Astar::toAddress(const int& x, const int& y, const int& z) {
//   return x * map_voxel_num_(1) * map_voxel_num_(2) + y * map_voxel_num_(2) + z;
// }

// int Astar::toAddress(const Eigen::Vector3i& id) {
//   return toAddress(id[0], id[1], id[2]);
// }

// int Astar::getInflateOccupancy(const Eigen::Vector3d& pos) {
//   Eigen::Vector3i id;
//   posToIndex(pos, id);
//   return getInflateOccupancy(id);
// }

// int Astar::getInflateOccupancy(const Eigen::Vector3i& id) {
//   if (!isInMap(id)) return -1;
//   return int(occupancy_buffer_inflate_[toAddress(id)]);
// }

// int Astar::getOccupancy(const Eigen::Vector3i& id) {
//   if (!isInMap(id)) return -1;
//   double occ = occupancy_buffer_[toAddress(id)];
//   if (occ < clamp_min_log_ - 1e-3) return UNKNOWN;
//   if (occ > min_occupancy_log_) return OCCUPIED;
//   return FREE;
// }

// }  // namespace fast_planner
