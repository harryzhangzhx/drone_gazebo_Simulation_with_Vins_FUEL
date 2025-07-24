#pragma once

#include <atomic>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

#include "controller_manager_types.h"
#include "path_planner/path_planner_api.h"
#include "controller_manager/topic_manager.h"
#include "controller_manager/service_manager.h"
#include "utilities/common_types.h"

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
        void handleNewTargetState();
        void handleExecutingState();
        void handleSafetyHoveringState();
        void handleLandingState();
        bool runGlobalPlanner();
        void runLocalPlanner();
        bool runGovernor();
        bool runMPC();
        void checkLoopTiming(const std::chrono::time_point<std::chrono::high_resolution_clock> &loop_start);

    private:
        // Node reference
        std::shared_ptr<rclcpp::Node> node_;

        // Thread control
        std::thread ctrl_thread_;
        std::atomic<bool> running_flag_;
        double ctrl_freq_;

        // State
        std::atomic<ctrl_manager::State> current_state_;
        Eigen::Vector4d current_target_;
        ctrl_manager::OdomWorld odom_world_;
        std::vector<Eigen::Vector4d> global_path_; // position + yaw
        std::vector<Eigen::Vector4d> local_path_;
        std::vector<double> voxel_map_;

        // Library instances
        std::shared_ptr<TopicManager> topic_manager_ptr_;
        std::shared_ptr<ServiceManager> service_manager_ptr_;
        // std::shared_ptr<MPC::MPC> mpc_ptr_;
    };

} // namespace ctrl_manager
