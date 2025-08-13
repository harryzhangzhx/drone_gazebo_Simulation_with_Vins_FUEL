#pragma once

#include <atomic>
#include <chrono>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

#include "controller_manager_types.h"
#include "path_planner/path_planner_api.h"
#include "governor/governor_api.h"
#include "controller/controller_api.h"
#include "utilities/common_types.h"
#include "utilities/voxel_map.h"
#include "controller_manager/topic_manager.h"
#include "controller_manager/service_manager.h"

namespace ctrl_manager
{

    class ControllerManager
    {
    public:
        ControllerManager(std::shared_ptr<rclcpp::Node> node, const utilities::AllParams &params);
        void startControlLoop();
        void start();
        void stop();

    private:
        void handleIdleState();
        bool handleNewTargetState();
        void handleExecutingState();
        void handleSafetyHoveringState();
        void handleLandingState();
        Eigen::VectorXd convertToState(const ctrl_manager::OdomWorld &odom_world);
        void checkLoopTiming(
            const std::chrono::time_point<std::chrono::steady_clock> &loop_start);

    private:
        // Node reference
        std::shared_ptr<rclcpp::Node> node_;

        // Thread control
        std::thread ctrl_thread_;
        std::atomic<bool> running_flag_;
        double ctrl_freq_ = 0.5; // TODO set with  yaml loader

        // State
        std::atomic<ctrl_manager::Status> current_status_;
        Eigen::Vector4d current_ref_; // position, yaw
        ctrl_manager::OdomWorld odom_world_;
        Eigen::VectorXd current_state_;            // position, position_dot, roll, pitch, yaw
        std::vector<Eigen::Vector3d> global_path_; // position and yaw
        std::vector<Eigen::Vector3d> local_path_;
        double s_;                  // auxiliary target reference
        Eigen::VectorXd ref_state_; // the equalibrium state at the reference
        Eigen::VectorXd optimal_decision_var_;
        Eigen::Vector4d ctrl_cmd_;

        // Library instances
        std::shared_ptr<TopicManager> topic_manager_ptr_;
        std::shared_ptr<ServiceManager> service_manager_ptr_;
        std::shared_ptr<VoxelMap> voxel_map_ptr_;
        std::uint64_t last_used_map_version_{0}; // Versioning for freshness checks
        std::shared_ptr<PathPlanner> path_planner_ptr_;
        std::shared_ptr<Controller> controller_ptr_;
        std::shared_ptr<Governor> governor_ptr_;
    };

} // namespace ctrl_manager
