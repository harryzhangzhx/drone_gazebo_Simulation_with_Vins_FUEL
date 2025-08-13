#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <iostream>
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include "utilities/voxel_map.h"

class PotentialField
{
public:
    std::vector<Eigen::Vector3d> path_;
    // enum{REACH_END = 1, NO_PATH = 2};

    PotentialField(const std::shared_ptr<VoxelMap> mapInput) : map_(mapInput)
    {
        mapInput->getRegion(origin_, map_size_3d_);
    }

    bool calcNavField(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

    void setRepulseVars(double influence_margin, double static_safety_margin, double collision_radius);

    void setSmoothingRadius(double smoothing_radius);
    void setGoalThreshold(double goal_threshold);
    void setPfStepSize(double pf_step_size);

    const std::vector<Eigen::Vector3d> &getPath();

    Eigen::Vector3d calcRep(const Eigen::Vector3d &cur_pt);

    Eigen::Vector3d calcAtt(const Eigen::Vector3d &cur_pt, const Eigen::Vector3d end_pt);

private:
    // double k_rep_;
    // double k_att_;
    double smoothing_radius_ = 1; // dont set to 0
    double influence_margin_ = 0.2;
    double static_safety_margin_ = 0.04;
    double collision_radius_;
    double pf_step_size_;
    double goal_threshold_;
    int MAX_ITER_ = 10000;

    // EDTEnvironment::Ptr edt_env_;

    const std::shared_ptr<VoxelMap> map_;
    Eigen::Vector3d origin_, map_size_3d_; // do we need this?
};
#endif