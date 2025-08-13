

#ifndef RRTSTAR_H
#define RRTSTAR_H

// your includes
#include <utilities/voxel_map.h>

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <random>
// #include <plan_env/edt_environment.h>

#include <cmath>
#include "KDTree.h"
#include <chrono>
#include <memory>
#include <vector>

struct rrtNode
{
public:
    Eigen::Vector3d position_;
    int index_;
    double cost_;
    int parent_;

    // Node(Eigen::Vector3d pos, Eigen::Vector3i ind, double c, Node* parent_) {
    //     position = pos;
    //     index = ind;
    //     cost = c;
    //     parent = parent_;
    // }
    // ~Node(){};
};
typedef rrtNode *rrtNodePtr;

class RRTstar
{
public:
    // KDTree kd_;
    enum
    {
        REACH_END = 1,
        NO_PATH = 2
    };
    std::vector<rrtNode> graph_;
    std::vector<Eigen::Vector3d> allPoints_;
    std::vector<Eigen::Vector3d> shortestPath_;
    // typedef std::pair<int. int> Edge;
    // std::vector<Edge> E;

    RRTstar(std::shared_ptr<VoxelMap> map) : map_(map)
    {
        map->getRegion(origin_, map_size_3d_);
        std::cout << "rrtstar map origin_: " << origin_.transpose() << std::endl;
        std::cout << "rrtstar map_size_3d_: " << map_size_3d_.transpose() << std::endl;
        resolution_ = map->map_params_.resolution_;
    }

    void setGoalRadius(double radius)
    {
        goal_radius_ = radius;
    }

    double getGoalRadius()
    {
        return goal_radius_;
    }
    void setResolution(const double &res);

    void tuneConstants(int g, int eta);

    bool search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

private:
    Eigen::Vector3d getRandPt();

    // int getNearestNode(const Node& new_node){ //implement a seperate data structure for better lookups?

    //     // std::size_t length{ graph_.size() };

    //     // int ind = 0;
    //     // double min_dist = distance(graph_[0], new_node);
    //     // for(std::size_t i{0}; i< length; i++){
    //     //     if(distance(graph_[i], new_node) < min_dist){
    //     //         ind = i;
    //     //         min_dist = distance(graph_[i], new_node);
    //     //     }
    //     // }
    //     return kd_.getNearestNode(new_node.position_);

    // }

    void rewireTree(const std::vector<rrtNode> &neighbours, const rrtNode &new_node);

    double distance(const rrtNode &node1, const rrtNode &node2);

    double distance(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
    std::vector<rrtNode> getNeighbours(const rrtNode &rrtNode);

    int getBestCost(const std::vector<rrtNode> &neighbours, const rrtNode &new_node);

    rrtNode steer(const rrtNode &node_nearest, const rrtNode &node_rand);

    bool collisionFree(const rrtNode &start, const rrtNode &end);

    void recoverPath();

    int isOccupied(Eigen::Vector3d &position);
    void resetKDTree();

    double goal_radius_;
    // double max_steer_dist;
    double g_;
    double eta_;
    // double neighbourhood;
    int MAX_ITER_ = 10000;
    rrtNode goal_node_;
    double resolution_;

    std::chrono::milliseconds timeLimit_{100}; // milliseconds

    std::shared_ptr<VoxelMap> map_;
    Eigen::Vector3d map_size_3d_, origin_;
};
#endif