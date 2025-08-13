#include "rrtstar.h"
#include <iostream>

void RRTstar::setResolution(const double &res)
{
    resolution_ = res;
}

void RRTstar::tuneConstants(int g, int eta)
{
    g_ = g;
    eta_ = eta;
}

bool RRTstar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
{
    goal_node_.cost_ = std::numeric_limits<double>::infinity();
    KDTree kd(start_pt);
    rrtNode init_node{start_pt, 0, 0.0, 0};
    graph_.push_back(init_node);

    auto timeStart = std::chrono::high_resolution_clock::now();

    int num = 1;
    bool timeOver = false;
    while (!timeOver)
    {

        rrtNode node_rand{getRandPt(), num, 0.0, -1};
        if (isOccupied(node_rand.position_))
        {
            continue;
        }

        int node_nearest = kd.getNearestNode(node_rand.position_);

        rrtNode new_node = steer(graph_[node_nearest], node_rand);
        std::vector<rrtNode> near_nodes;
        if (getNeighbours(new_node).empty())
        {
            near_nodes = {graph_[node_nearest]};
        }
        else
        {
            near_nodes = getNeighbours(new_node);
        }

        int node_min = getBestCost(near_nodes, new_node); // Choose optimal cost_ parent

        new_node.parent_ = node_min;
        new_node.index_ = num;
        new_node.cost_ = graph_[node_min].cost_ + distance(graph_[node_min], new_node);
        graph_.push_back(new_node);
        kd.insertNode(new_node.position_, num);
        // E.append(std::make_pair(new_node.index, node_min.index));

        rewireTree(near_nodes, new_node);
        num++;
        if (distance(new_node.position_, end_pt) < goal_radius_ && new_node.index_ != 0) // Have to see how we will implement our final check
        {
            if (new_node.cost_ < goal_node_.cost_)
            {
                goal_node_ = new_node;
                // goal_node_.index_ = new_node.index_;
            }
        }
        else if (num > MAX_ITER_)
        {
            return false;
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - timeStart);
        if (elapsed >= timeLimit_)
        {
            timeOver = true;
        }
    }
    if (goal_node_.index_ != 0)
    {
        recoverPath();
        return true;
    }
    else
    {

        return false;
    }
}

Eigen::Vector3d RRTstar::getRandPt()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    double x = map_size_3d_(0) / 2;
    double y = map_size_3d_(1) / 2;
    double z = map_size_3d_(2) / 2;
    Eigen::Vector3d vec;
    // vec << std::uniform_real_distribution<double>(-x, x), std::uniform_real_distribution<double>(-y, y), std::uniform_real_distribution<double>(-z, z)(gen);
    std::uniform_real_distribution<double> dist_x(0, x * 2);
    std::uniform_real_distribution<double> dist_y(0, y * 2);
    std::uniform_real_distribution<double> dist_z(0, z * 2);
    vec << dist_x(gen), dist_y(gen), dist_z(gen);

    return vec;
}

// int getNearestNode(const rrtNode& new_node){ //implement a seperate data structure for better lookups?

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

//}

void RRTstar::rewireTree(const std::vector<rrtNode> &neighbours, const rrtNode &new_node)
{
    std::size_t length{neighbours.size()};
    for (std::size_t i{0}; i < length; i++)
    {
        if (collisionFree(neighbours[i], new_node) && new_node.cost_ + distance(new_node, neighbours[i]) < neighbours[i].cost_ && neighbours[i].index_ != new_node.index_)
        {
            graph_[neighbours[i].index_].parent_ = new_node.index_;
            graph_[neighbours[i].index_].cost_ = new_node.cost_ + distance(new_node, graph_[neighbours[i].index_]);
        }
    }
}

double RRTstar::distance(const rrtNode &node1, const rrtNode &node2)
{
    return (node1.position_ - node2.position_).norm();
}

double RRTstar::distance(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    return (end - start).norm();
}

std::vector<rrtNode> RRTstar::getNeighbours(const rrtNode &node)
{
    std::vector<rrtNode> neighbours;
    std::size_t length{graph_.size()};
    for (std::size_t i{0}; i < length; i++)
    {
        if (distance(graph_[i], node) < std::min(g_ * std::pow((std::log(length) / length), 1.0 / 3.0), eta_))
        {
            neighbours.push_back(graph_[i]);
        }
    }
    return neighbours;
}

int RRTstar::getBestCost(const std::vector<rrtNode> &neighbours, const rrtNode &new_node)
{

    if (neighbours.empty())
    {
        std::cerr << "getBestCost called with empty node list!" << std::endl;
        return std::numeric_limits<double>::infinity(); // or some safe fallback
    }

    std::size_t length{neighbours.size()};

    int ind = 0;

    double cost = neighbours[0].cost_ + distance(new_node, neighbours[0]);
    for (std::size_t i{0}; i < length; i++)
    {
        if (neighbours[i].cost_ + distance(neighbours[i], new_node) < cost && collisionFree(neighbours[i], new_node))
        {
            cost = neighbours[i].cost_ + distance(neighbours[i], new_node);
            ind = i;
        }
    }

    // std::cout << "Best cost index: " << neighbours[ind].index_ << std::endl;
    return neighbours[ind].index_;
}

rrtNode RRTstar::steer(const rrtNode &node_nearest, const rrtNode &node_rand)
{

    if (distance(node_nearest, node_rand) <= eta_)
    {
        return node_rand;
    }
    Eigen::Vector3d pos = node_nearest.position_ + eta_ * (node_rand.position_ - node_nearest.position_) / ((node_rand.position_ - node_nearest.position_).norm());

    rrtNode new_node{pos, 0, 0.0, -1};
    return new_node;
}

bool RRTstar::collisionFree(const rrtNode &start, const rrtNode &end)
{
    // need to test if the line between two nodes is completely unobstructed
    // could follow along the line and then see if it get obstrcuted, or can go through every object
    // for big maps it wouldnt work, to go through every object

    // implementing digital differential analyzer (3D-DDA) outlined by Fast Voxel Traversal Algorithm
    // assume that all nodes take place in one occupied voxel

    double x = std::floor(start.position_[0] / resolution_) * resolution_;
    double y = std::floor(start.position_[1] / resolution_) * resolution_;
    double z = std::floor(start.position_[2] / resolution_) * resolution_;

    double Xmax = std::floor(end.position_[0] / resolution_) * resolution_;
    double Ymax = std::floor(end.position_[1] / resolution_) * resolution_;
    double Zmax = std::floor(end.position_[2] / resolution_) * resolution_;

    Eigen::Vector3d maxVec;
    maxVec << Xmax, Ymax, Zmax;
    // double dist = distance(start, end);

    double stepX = (Xmax > x) ? 1.0 : (Xmax < x ? -1.0 : 0.0);
    double stepY = (Ymax > y) ? 1.0 : (Ymax < y ? -1.0 : 0.0);
    double stepZ = (Zmax > z) ? 1.0 : (Zmax < z ? -1.0 : 0.0);

    Eigen::Vector3d diff = start.position_ - end.position_;
    double dx = diff[0];
    double dy = diff[1];
    double dz = diff[2];

    double voxelBorderDistX = (stepX < 0) ? fmod(start.position_[0], resolution_) : resolution_ - fmod(start.position_[0], resolution_);
    double voxelBorderDistY = (stepY < 0) ? fmod(start.position_[1], resolution_) : resolution_ - fmod(start.position_[1], resolution_);
    double voxelBorderDistZ = (stepZ < 0) ? fmod(start.position_[2], resolution_) : resolution_ - fmod(start.position_[2], resolution_);

    double tMaxX = (stepX != 0) ? voxelBorderDistX / std::abs(dx) : std::numeric_limits<double>::infinity();
    double tMaxY = (stepY != 0) ? voxelBorderDistY / std::abs(dy) : std::numeric_limits<double>::infinity();
    double tMaxZ = (stepZ != 0) ? voxelBorderDistZ / std::abs(dz) : std::numeric_limits<double>::infinity();

    double tDeltaX = (stepX != 0) ? resolution_ / std::abs(dx) : std::numeric_limits<double>::infinity();
    double tDeltaY = (stepY != 0) ? resolution_ / std::abs(dy) : std::numeric_limits<double>::infinity();
    double tDeltaZ = (stepZ != 0) ? resolution_ / std::abs(dz) : std::numeric_limits<double>::infinity();

    // std::cout << "Start point: " << x <<", " << y << ", " << z << std::endl;
    // std::cout << "End point: " << Xmax <<", " << Ymax << ", " << Zmax  << std::endl;

    Eigen::Vector3d vec;
    vec << x, y, z;

    // Eigen::Vector3i voxel = (vec / resolution_).array().floor().cast<int>();
    //  Eigen::Vector3i target_voxel = (maxVec / resolution_).array().floor().cast<int>();

    while ((vec - maxVec).squaredNorm() > 0.03)
    {
        vec << x, y, z;
        int occ = isOccupied(vec);
        if (occ == -1)
        {
            // std::cout << "Collision free, reached end of map!" << std::endl;
            return true;
        }
        else if (occ == 1)
        {
            // std::cout << "Collision detected at: " << vec.transpose() << std::endl;
            return false;
        }

        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                x += stepX * resolution_;
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ * resolution_;
                tMaxZ += tDeltaZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                y += stepY * resolution_;
                tMaxY += tDeltaY;
            }
            else
            {
                z += stepZ * resolution_;
                tMaxZ += tDeltaZ;
            }
        }
        // std::cout << "Current position: " << vec.transpose() << std::endl;
    }

    // std::cout << "Collision free!" << std::endl;
    return true;
}

void RRTstar::recoverPath()
{
    rrtNode &cur_node = goal_node_;
    // std::vector<Eigen::Vector3d> path;
    shortestPath_.clear();
    while (cur_node.parent_ != 0)
    {
        shortestPath_.push_back(cur_node.position_);
        cur_node = graph_[cur_node.parent_];
    }
    shortestPath_.push_back(graph_[0].position_);
}
int RRTstar::isOccupied(Eigen::Vector3d &position_)
{
    return map_->getInflateOccupancy(position_);
}
