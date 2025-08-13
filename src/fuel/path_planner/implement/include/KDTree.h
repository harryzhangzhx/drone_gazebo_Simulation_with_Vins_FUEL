#pragma once

#include <memory>
#include <limits>
#include <Eigen/Dense>

class KDTree
{
public:
    KDTree(const Eigen::Vector3d &root);               // Constructor
    int getNearestNode(const Eigen::Vector3d &target); // NN search

    struct KDNode
    {
        std::unique_ptr<KDNode> left_;
        std::unique_ptr<KDNode> right_;
        Eigen::Vector3d position_;
        int index_;

        KDNode(const Eigen::Vector3d &pos,
               std::unique_ptr<KDNode> left = nullptr,
               std::unique_ptr<KDNode> right = nullptr,
               int index = -1);
    };

    using KDNodePtr = KDNode *;

    void insertNode(const Eigen::Vector3d &input, int num);
    void getNearestNodeHelper(KDNodePtr curNode,
                              const Eigen::Vector3d &target,
                              KDNodePtr &bestNode,
                              double &bestDist,
                              int depth);
    void reset();
    double getDistance(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

private:
    std::unique_ptr<KDNode> root_;
};