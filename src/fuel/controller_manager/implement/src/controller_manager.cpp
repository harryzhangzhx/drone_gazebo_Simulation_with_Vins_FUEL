#include "controller_manager.h"
#include "controller_manager/topic_manager.h"
#include <thread>
#include <atomic>
#include <mutex>

namespace ctrl_manager
{

    ControllerManager::ControllerManager(std::shared_ptr<rclcpp::Node> node, const utilities::AllParams &params)
        : node_(node),
          current_state_(ctrl_manager::NEW_TARGET),
          running_flag_(false)
    {
        topic_manager_ptr_ = std::make_shared<TopicManager>(node);
        service_manager_ptr_ = std::make_shared<ServiceManager>(node);
        // mpc_ptr_ = std::make_shared<MPC::MPC>(params);
    }

    void ControllerManager::start()
    {
        running_flag_ = true;
        ctrl_thread_ = std::thread(&ControllerManager::startControlLoop, this);
    }

    void ControllerManager::stop()
    {
        running_flag_ = false;
        if (ctrl_thread_.joinable())
        {
            ctrl_thread_.join();
        }
    }

    void ControllerManager::startControlLoop()
    {
        rclcpp::Rate rate(ctrl_freq_);
        RCLCPP_INFO(node_->get_logger(), "[FSM] Started at %.1f Hz", ctrl_freq_);

        while (running_flag_ && rclcpp::ok())
        {
            auto loop_start = std::chrono::high_resolution_clock::now();

            if (topic_manager_ptr_->getNewTargetFlag())
            {
                current_state_ = ctrl_manager::NEW_TARGET;
            }

            // switch (current_state_.load())
            // {
            // case ctrl_manager::IDLE:
            //     handleIdleState();
            //     break;
            // case ctrl_manager::NEW_TARGET:
            //     handleNewTargetState();
            //     break;
            // case ctrl_manager::EXECUTING:
            //     handleExecutingState();
            //     break;
            // case ctrl_manager::SAFETY_HOVERING:
            //     handleSafetyHoveringState();
            //     break;
            // case ctrl_manager::LANDING:
            //     handleLandingState();
            //     break;
            // }

            // Timing safety check
            checkLoopTiming(loop_start);
            rate.sleep();
        }

        RCLCPP_INFO(node_->get_logger(), "[FSM] Stopped");
    }

    void ControllerManager::handleIdleState() {}

    void ControllerManager::handleNewTargetState()
    {
        current_target_ = topic_manager_ptr_->getNewTarget();

        // global planning
        if (runGlobalPlanner())
        {
            topic_manager_ptr_->setNewTargetFlag(false);
        }
    }

    void ControllerManager::handleExecutingState() {}
    void ControllerManager::handleSafetyHoveringState() {}
    void ControllerManager::handleLandingState() {}

    bool ControllerManager::runGlobalPlanner()
    {
        odom_world_ = topic_manager_ptr_->getOdom();
        voxel_map_ = service_manager_ptr_->getInflatedMap();
        bool global_planner_flag = false;
        // global_planner_flag = getRRTPath(odom_world, current_target_, voxel_map, global_path_);

        if (global_planner_flag)
        {
            RCLCPP_INFO(node_->get_logger(), "global path planner: Path found");
            return true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "global path planner: Path NOT found");
            return false;
        }
    }

    void ControllerManager::runLocalPlanner() {}
    bool ControllerManager::runGovernor() { return true; }
    bool ControllerManager::runMPC() { return true; }

    void ControllerManager::checkLoopTiming(const std::chrono::time_point<std::chrono::high_resolution_clock> &loop_start)
    {
        auto loop_end = std::chrono::high_resolution_clock::now();
        double loop_time = std::chrono::duration<double>(loop_end - loop_start).count();

        if (loop_time > 1.0 / ctrl_freq_)
        {
            RCLCPP_WARN(node_->get_logger(), "[FSM] Loop took %.3fms (expected <%.3fms)",
                        loop_time * 1000, (1.0 / ctrl_freq_) * 1000);

            if (loop_time > 1.0 / ctrl_freq_ * 0.8)
            {
                current_state_ = ctrl_manager::SAFETY_HOVERING;
                RCLCPP_ERROR(node_->get_logger(), "[FSM] CRITICAL OVERRUN -> SAFETY_HOVERING");
            }
        }
    }

} // namespace ctrl_manager
