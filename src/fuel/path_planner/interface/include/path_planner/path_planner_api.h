#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include "utilities/voxel_map.h"
#include "utilities/common_types.h"

class Astar;
class RRTstar;
class PotentialField;

class PathPlanner
{
public:
    PathPlanner(const utilities::AllParams &params, std::shared_ptr<VoxelMap> map);
    bool getRRTPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path);
    bool getPotentialFieldPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path);
    bool getAstarPath(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d> &path); // do I need map data, i think a pointer of the map could be put here in the class?
    void setRRTStarParams(double g, double eta);
    void setPotentialFieldParams(double smoothing_radius, double influence_margin, double static_safety_margin, double collision_radius);
    // void setAStarParams(const ); // to be finished, idk what parameters they have
private:
    utilities::AllParams params_;
    std::shared_ptr<VoxelMap> map_; // have to make sure that this is thread safe when functions are modifying this
    std::vector<Eigen::Vector3d> path_;
    std::shared_ptr<RRTstar> rrtstar_ptr_;
    std::shared_ptr<Astar> astar_ptr_;
    std::shared_ptr<PotentialField> potential_field_ptr_;
};
