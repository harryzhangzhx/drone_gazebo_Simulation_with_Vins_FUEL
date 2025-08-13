
#include "KDTree.h"

KDTree::KDTree(const Eigen::Vector3d &root)
{
    root_ = std::make_unique<KDNode>(root, nullptr, nullptr, 0);
}

KDTree::KDNode::KDNode(const Eigen::Vector3d &pos,
                       std::unique_ptr<KDNode> left,
                       std::unique_ptr<KDNode> right,
                       int index)
    : position_(pos), left_(std::move(left)), right_(std::move(right)), index_(index) {}

int KDTree::getNearestNode(const Eigen::Vector3d &target)
{
    KDNodePtr bestNode = nullptr;
    double bestDist = std::numeric_limits<double>::infinity();
    getNearestNodeHelper(root_.get(), target, bestNode, bestDist, 0);
    return bestNode->index_;
}

void KDTree::insertNode(const Eigen::Vector3d &input, int num)
{
    int depth = 0;
    KDNodePtr cur_node = root_.get();

    auto newNode = std::make_unique<KDNode>(input, nullptr, nullptr, num);

    if (root_ == nullptr)
    {
        root_ = std::move(newNode);
        return;
    }

    while (true)
    {
        int index = depth % 3;
        if (cur_node->position_[index] > input[index])
        {
            if (cur_node->left_ == nullptr)
            {
                cur_node->left_ = std::move(newNode);
                break;
            }
            cur_node = cur_node->left_.get();
        }
        else
        {
            if (cur_node->right_ == nullptr)
            {
                cur_node->right_ = std::move(newNode);
                break;
            }
            cur_node = cur_node->right_.get();
        }
        ++depth;
    }
}

void KDTree::getNearestNodeHelper(KDNodePtr curNode,
                                  const Eigen::Vector3d &target,
                                  KDNodePtr &bestNode,
                                  double &bestDist,
                                  int depth)
{
    if (!curNode)
        return;

    double dist = getDistance(target, curNode->position_);
    if (dist < bestDist)
    {
        bestDist = dist;
        bestNode = curNode;
    }

    int index = depth % 3;
    KDNodePtr next = nullptr;
    KDNodePtr other = nullptr;

    if (curNode->position_[index] > target[index])
    {
        next = curNode->left_.get();
        other = curNode->right_.get();
    }
    else
    {
        next = curNode->right_.get();
        other = curNode->left_.get();
    }

    if (next)
    {
        getNearestNodeHelper(next, target, bestNode, bestDist, depth + 1);
    }

    if (std::abs(curNode->position_[index] - target[index]) < bestDist && other)
    {
        getNearestNodeHelper(other, target, bestNode, bestDist, depth + 1);
    }
}

void KDTree::reset()
{
    root_.reset();
}

double KDTree::getDistance(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    return (end - start).squaredNorm();
}