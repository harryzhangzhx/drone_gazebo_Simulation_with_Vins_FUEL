#include "potential_field.h"

bool PotentialField::calcNavField(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt){
    Eigen::Vector3d cur_pt = start_pt;
    path_.push_back(cur_pt);
    int ite = 0;
    while((cur_pt - end_pt).norm() > goal_threshold_){
        
        Eigen::Vector3d new_pt = cur_pt + pf_step_size_ * (calcAtt(cur_pt, end_pt) + calcRep(cur_pt));
        path_.push_back(new_pt);
        cur_pt = new_pt;

        ++ite;

        if(ite >= MAX_ITER_){
            return false;
        }
    }
    return true;
    
}

void PotentialField::setRepulseVars(double influence_margin, double static_safety_margin, double collision_radius){
    influence_margin_ = influence_margin;
    static_safety_margin_ = static_safety_margin;
    collision_radius_ = collision_radius;
}

void PotentialField::setSmoothingRadius(double smoothing_radius){
    smoothing_radius_ = smoothing_radius;
}

void PotentialField::setGoalThreshold(double goal_threshold){
    goal_threshold_ = goal_threshold;
}

void PotentialField::setPfStepSize(double pf_step_size){
    pf_step_size_ = pf_step_size;
}

const std::vector<Eigen::Vector3d>& PotentialField::getPath() {
    return path_;
}


Eigen::Vector3d PotentialField::calcRep(const Eigen::Vector3d& cur_pt) {

    Eigen::Vector3d grad;
    double signed_distance = map_->getDistWithGrad(cur_pt, grad);

    Eigen::Vector3d repulse_walls;
    repulse_walls << 0.0, 0.0, 0.0;
    if(signed_distance - collision_radius_ < influence_margin_) {
        repulse_walls = -1*std::max((influence_margin_ - (signed_distance - collision_radius_))/(influence_margin_ - static_safety_margin_), 0.0)*grad;
    }
    
    return repulse_walls;
}

Eigen::Vector3d PotentialField::calcAtt(const Eigen::Vector3d& cur_pt, const Eigen::Vector3d end_pt){
    return (end_pt-cur_pt)/std::max((end_pt-cur_pt).norm(), smoothing_radius_);
}




