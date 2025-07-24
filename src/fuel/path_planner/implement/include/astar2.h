// #ifndef _ASTAR2_H
// #define _ASTAR2_H

// #include <Eigen/Core>
// #include <iostream>
// #include <map>
// #include <ros/console.h>
// #include <ros/ros.h>
// #include <string>
// #include <unordered_map>
// #include <plan_env/edt_environment.h>
// #include <plan_env/sdf_map.h>
// #include <plan_env/map_ros.h>
// #include <boost/functional/hash.hpp>
// #include <queue>
// #include "matrix_hash.h"

// namespace path_planner {
// class Node {
// public:
//   Eigen::Vector3i index;
//   Eigen::Vector3d position;
//   double g_score, f_score;
//   Node* parent;

//   /* -------------------- */
//   Node() {
//     parent = NULL;
//   }
//   ~Node(){};
// };
// typedef Node* NodePtr;

// class NodeComparator0 {
// public:
//   bool operator()(NodePtr node1, NodePtr node2) {
//     return node1->f_score > node2->f_score;
//   }
// };

// class Astar {
// public:
//   Astar();
//   ~Astar();

//   void init(ros::NodeHandle& nh, const fast_planner::EDTEnvironment::Ptr& env);
//   void reset();
//   int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
//   void setResolution(const double& res);
//   static double pathLength(const vector<Eigen::Vector3d>& path);

//   std::vector<Eigen::Vector3d> getPath();
//   std::vector<Eigen::Vector3d> getVisited();
//   double getEarlyTerminateCost();

//   double lambda_heu_;
//   double max_search_time_;

// private:
//   void backtrack(const NodePtr& end_node, const Eigen::Vector3d& end);
//   void posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx);
//   double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
//   double getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
//   double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

//   bool isInBox(const Eigen::Vector3i& id);
//   bool isInMap(const Eigen::Vector3d& pos);
//   int toAddress(const int& x, const int& y, const int& z);
//   int toAddress(const Eigen::Vector3i& id);
//   int getInflateOccupancy(const Eigen::Vector3d& pos);
//   int getInflateOccupancy(const Eigen::Vector3i& id);
//   int getOccupancy(const Eigen::Vector3i& id);

//   // main data structure
//   vector<NodePtr> path_node_pool_;
//   int use_node_num_, iter_num_;
//   std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
//   std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
//   std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
//   std::vector<Eigen::Vector3d> path_nodes_;
//   double early_terminate_cost_;

//   fast_planner::EDTEnvironment::Ptr edt_env_;

//   // parameter
//   double margin_;
//   int allocate_num_;
//   double tie_breaker_;
//   double resolution_, inv_resolution_;
//   Eigen::Vector3d map_min_boundary_, map_max_boundary_;
//   Eigen::Vector3d map_size_3d_, origin_;
//   Eigen::Vector3i box_min_, box_max_;
//   Eigen::Vector3i map_voxel_num_;
//   double clamp_min_log_, min_occupancy_log_;

//   //map data
//   std::vector<double> occupancy_buffer_;
//   std::vector<char> occupancy_buffer_inflate_;
// };

// }  // namespace

// #endif