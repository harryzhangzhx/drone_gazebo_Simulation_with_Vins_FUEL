#include "controller_manager.h"
#include "controller_manager/topic_manager.h"
#include <thread>
#include <atomic>
#include <mutex>

namespace ctrl_manager
{

    ControllerManager::ControllerManager(std::shared_ptr<rclcpp::Node> node, const utilities::AllParams &params)
        : node_(node),
          current_status_(ctrl_manager::NEW_TARGET),
          running_flag_(false)
    {
        // initialize ros topic and service managers
        topic_manager_ptr_ = std::make_shared<TopicManager>(node);
        service_manager_ptr_ = std::make_shared<ServiceManager>(node);

        // request map data and parameters
        voxel_map_ptr_ = std::make_shared<VoxelMap>();
        service_manager_ptr_->updateMapParam(voxel_map_ptr_);
        service_manager_ptr_->updateMapData(voxel_map_ptr_);
        RCLCPP_INFO(node_->get_logger(), "VoxelMap instantiated");

        // initialize the path planner, controller and governor
        path_planner_ptr_ = std::make_shared<PathPlanner>(params, voxel_map_ptr_);
        RCLCPP_INFO(node_->get_logger(), "Path planner instantiated");
        controller_ptr_ = std::make_shared<Controller>(params, voxel_map_ptr_);
        RCLCPP_INFO(node_->get_logger(), "Controller instantiated");

        governor_ptr_ = std::make_shared<Governor>(params, voxel_map_ptr_);
        RCLCPP_INFO(node_->get_logger(), "Governor instantiated");
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
        int loop_count = 0;

        while (running_flag_ && rclcpp::ok())
        {
            RCLCPP_INFO(node_->get_logger(), "Control loop_count %d", loop_count++);

            auto loop_start = std::chrono::steady_clock::now();

            odom_world_ = topic_manager_ptr_->getOdom();

            current_state_ = convertToState(odom_world_);

            service_manager_ptr_->updateMapData(voxel_map_ptr_);
            if (service_manager_ptr_->hasNewMap(last_used_map_version_))
            {
                last_used_map_version_ = service_manager_ptr_->currentVersion();
                RCLCPP_INFO(node_->get_logger(), "map update with version no. %d", last_used_map_version_);
            }

            if (topic_manager_ptr_->getNewTargetFlag())
            {
                current_status_ = ctrl_manager::NEW_TARGET;
            }

            switch (current_status_.load())
            {
            case ctrl_manager::IDLE:
                handleIdleState();
                break;
            case ctrl_manager::NEW_TARGET:
                handleNewTargetState();
                break;
            case ctrl_manager::EXECUTING:
                handleExecutingState();
                break;
            case ctrl_manager::SAFETY_HOVERING:
                handleSafetyHoveringState();
                break;
            case ctrl_manager::LANDING:
                handleLandingState();
                break;
            default:
                break;
            }

            // compute local path
            path_planner_ptr_->getPotentialFieldPath(current_state_.head<3>(), current_ref_.head<3>(), local_path_);

            // compute the next auxiliary target reference
            // TODO currently use the current state, later change to the last predicted state from MPC
            double prior_s = s_;
            std::tie(s_, ref_state_) = governor_ptr_->getGovernorRef(local_path_, current_state_, prior_s);

            // // Use MPC to compute the optimal states and control inputs
            // optimal_decision_var_ = controller_ptr_->getMPCCommand(current_state_, ref_state_);
            // ctrl_cmd_ = optimal_decision_var_.segment(9, 4); // extract the optimal control command

            // // Use LQR to compute the optimal control inputs
            // ctrl_cmd_ = controller_ptr_->getLQRCommand(current_state_, ref_state_);

            // publish control command (thrust, omega)
            ctrl_cmd_ = Eigen::Vector4d(5.0, 0.0, 0.0, 0.0); // dummy command for testing
            topic_manager_ptr_->publishThrustAndOmega(ctrl_cmd_);

            checkLoopTiming(loop_start);

            rate.sleep();
        }

        RCLCPP_INFO(node_->get_logger(), "[FSM] Stopped");
    }

    bool ControllerManager::handleNewTargetState()
    {
        current_ref_ = topic_manager_ptr_->getNewTarget();

        bool global_planner_flag = path_planner_ptr_->getRRTPath(current_state_.head<3>(), current_ref_.head<3>(), global_path_);

        if (global_planner_flag)
        {
            topic_manager_ptr_->setNewTargetFlag(false);
            return true;
        }
        return false;
    }

    void ControllerManager::handleIdleState() {}
    void ControllerManager::handleExecutingState() {}
    void ControllerManager::handleSafetyHoveringState() {}
    void ControllerManager::handleLandingState() {}

    Eigen::VectorXd ControllerManager::convertToState(const ctrl_manager::OdomWorld &odom_world)
    {
        Eigen::VectorXd odom_vec(9);
        odom_vec.head<3>() = odom_world.odom_pos; // position

        odom_vec.segment<3>(3) = odom_world.odom_vel; // velocity

        odom_vec.tail<3>() = odom_world.odom_orient.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
        return odom_vec;
    }

    void ControllerManager::checkLoopTiming(
        const std::chrono::time_point<std::chrono::steady_clock> &loop_start)
    {
        using clock = std::chrono::steady_clock;

        const double period_s = 1.0 / ctrl_freq_;
        const auto loop_end = clock::now();
        const double loop_time_s =
            std::chrono::duration<double>(loop_end - loop_start).count();

        const double utilization = loop_time_s / period_s;

        if (utilization >= 1.0)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[FSM] Loop took %.3f ms (period %.3f ms) — OVERRUN",
                        loop_time_s * 1000.0, period_s * 1000.0);
            current_status_ = ctrl_manager::NEW_TARGET; // TODO update to safety hovering state once implemented
            RCLCPP_ERROR(node_->get_logger(), "[FSM] CRITICAL OVERRUN -> SAFETY_HOVERING");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[FSM] Loop at %.0f%% of budget (%.3f / %.3f ms)",
                        utilization * 100.0, loop_time_s * 1000.0, period_s * 1000.0);
        }
    }

} // namespace ctrl_manager
