#include "controller_manager/service_manager.h"
#include <chrono>
#include <memory>
#include <iostream>
#include <mutex>

using namespace std::chrono_literals;

namespace ctrl_manager
{

    ServiceManager::ServiceManager(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {

        // map_data_client_ = node->create_client<quadrotor_msgs::srv::GetMapData>("map_data_service");
        // map_param_client_ = node->create_client<quadrotor_msgs::srv::GetMapParam>("map_param_service");

        map_data_client_ = node->create_client<quadrotor_msgs::srv::GetMapData>(
            "/get_map_Data");
        map_param_client_ = node->create_client<quadrotor_msgs::srv::GetMapParam>(
            "/get_map_param");
    }

    // void ServiceManager::updateMapData(std::shared_ptr<VoxelMap> voxel_map_ptr)
    // {
    //     if (!map_data_client_->service_is_ready())
    //     {
    //         RCLCPP_WARN(node_->get_logger(), "get_map_Data not ready; skipping request");
    //         return;
    //     }

    //     // A request is already in flight; do not queue another.
    //     bool expected = false;
    //     if (!request_in_flight_.compare_exchange_strong(expected, true))
    //     {
    //         return;
    //     }

    //     auto request = std::make_shared<quadrotor_msgs::srv::GetMapData::Request>();

    //     map_data_client_->async_send_request(
    //         request,
    //         [this, voxel_map_ptr](
    //             rclcpp::Client<quadrotor_msgs::srv::GetMapData>::SharedFuture future)
    //         {
    //             this->onMapDataResponse(std::move(future), voxel_map_ptr);
    //         });
    // }

    // //  TODO use MultiThreadedExecutor (≥2 threads) so the service callback can run in parallel with other node callbacks (timers, subs)
    // void ServiceManager::onMapDataResponse(
    //     rclcpp::Client<quadrotor_msgs::srv::GetMapData>::SharedFuture future,
    //     std::shared_ptr<VoxelMap> voxel_map_ptr)
    // {
    //     request_in_flight_.store(false);

    //     try
    //     {
    //         quadrotor_msgs::srv::GetMapData::Response::SharedPtr resp_ptr = future.get();

    //         std::size_t occ_sz = resp_ptr->occupancy_buffer.size();
    //         std::size_t inf_sz = resp_ptr->occupancy_buffer_inflate.size();
    //         std::size_t dis_sz = resp_ptr->distance_buffer.size();
    //         std::size_t disneg_sz = resp_ptr->distance_buffer_neg.size();

    //         {
    //             std::lock_guard<std::mutex> g(voxel_map_ptr->mtx_);
    //             voxel_map_ptr->occupancy_buffer_ = std::move(resp_ptr->occupancy_buffer);
    //             voxel_map_ptr->occupancy_buffer_inflate_ = std::move(resp_ptr->occupancy_buffer_inflate);
    //             voxel_map_ptr->distance_buffer_ = std::move(resp_ptr->distance_buffer);
    //             voxel_map_ptr->distance_buffer_neg_ = std::move(resp_ptr->distance_buffer_neg);
    //         }

    //         {
    //             std::lock_guard<std::mutex> lk(version_mtx_);
    //             ++map_version_;
    //         }

    //         RCLCPP_INFO(node_->get_logger(),
    //                     "Async MapData applied (version=%llu, occupancy=%zu, occupancy_inflate=%zu, dis=%zu, dis_negative=%zu)",
    //                     static_cast<unsigned long long>(map_version_),
    //                     occ_sz,
    //                     inf_sz,
    //                     dis_sz,
    //                     disneg_sz);
    //     }
    //     catch (const std::exception &e)
    //     {
    //         RCLCPP_ERROR(node_->get_logger(), "Async MapData failed: %s", e.what());
    //     }
    // }

    bool ServiceManager::hasNewMap(std::uint64_t since_version) const
    {
        std::lock_guard<std::mutex> lk(version_mtx_);
        return map_version_ > since_version;
    }

    std::uint64_t ServiceManager::currentVersion() const
    {
        std::lock_guard<std::mutex> lk(version_mtx_);
        return map_version_;
    }

    // Synchronous version of updateMapData with timeout and caching
    bool ServiceManager::updateMapData(std::shared_ptr<VoxelMap> voxel_map_ptr)
    {
        std::chrono::milliseconds timeout = std::chrono::milliseconds(3000);

        auto req = std::make_shared<quadrotor_msgs::srv::GetMapData::Request>();

        if (!map_data_client_->wait_for_service(1s))
        {
            std::lock_guard<std::mutex> lk(cache_mtx_);
            if (!cache_valid_)
            {
                RCLCPP_ERROR(node_->get_logger(), "get_map_Data unavailable and no cache yet");
                return false;
            }
            voxel_map_ptr->occupancy_buffer_ = cache_raw_;
            voxel_map_ptr->occupancy_buffer_inflate_ = cache_inflate_;
            RCLCPP_WARN(node_->get_logger(), "Using cached map (service unavailable)");
            return true;
        }

        auto fut = map_data_client_->async_send_request(req);

        auto deadline = std::chrono::steady_clock::now() + timeout;

        // if (rclcpp::spin_until_future_complete(node_, fut) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "MapData succeed: spin_until_future_complete");
        // }

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            if (fut.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
            {
                RCLCPP_INFO(node_->get_logger(), "future_status::ready");
                break;
            }

            RCLCPP_WARN(node_->get_logger(), "wait_for timed out, no response yet");
        }

        if (fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
            std::lock_guard<std::mutex> lk(cache_mtx_);
            if (!cache_valid_)
            {
                RCLCPP_WARN(node_->get_logger(), "MapData timeout and no cache yet");
                return false;
            }
            voxel_map_ptr->occupancy_buffer_ = cache_raw_;
            voxel_map_ptr->occupancy_buffer_inflate_ = cache_inflate_;
            RCLCPP_WARN(node_->get_logger(), "MapData timed out, used cached map");
            return true;
        }

        // Fresh response
        auto resp = fut.get();
        voxel_map_ptr->occupancy_buffer_ = resp->occupancy_buffer;
        voxel_map_ptr->occupancy_buffer_inflate_.assign(
            resp->occupancy_buffer_inflate.begin(),
            resp->occupancy_buffer_inflate.end());
        voxel_map_ptr->distance_buffer_.assign(
            resp->distance_buffer.begin(),
            resp->distance_buffer.end());
        voxel_map_ptr->distance_buffer_neg_.assign(
            resp->distance_buffer_neg.begin(),
            resp->distance_buffer_neg.end());

        // Update cache
        std::lock_guard<std::mutex> lk(cache_mtx_);
        cache_raw_ = voxel_map_ptr->occupancy_buffer_;
        cache_inflate_ = voxel_map_ptr->occupancy_buffer_inflate_;
        cache_distance_ = voxel_map_ptr->distance_buffer_;
        cache_distance_neg_ = voxel_map_ptr->distance_buffer_neg_;
        cache_valid_ = true;

        RCLCPP_INFO(node_->get_logger(), "MapData updated: raw=%zu inflate=%zu",
                    cache_raw_.size(), cache_inflate_.size());
        // RCLCPP_INFO(node_->get_logger(), "MapData updated: cache_raw[0] = %f", cache_raw_[0]);
        // RCLCPP_INFO(node_->get_logger(), "MapData updated: cache_inflate[0] = %d", cache_inflate_[0]);
        // RCLCPP_INFO(node_->get_logger(), "MapData updated: cache_distance[0] = %f", cache_distance_[0]);
        // RCLCPP_INFO(node_->get_logger(), "MapData updated: cache_distance_neg[0] = %f", cache_distance_neg_[0]);
        return true;
    }

    bool ServiceManager::updateMapParam(std::shared_ptr<VoxelMap> voxel_map_ptr)
    {
        VoxelMapParam map_param;
        auto request = std::make_shared<quadrotor_msgs::srv::GetMapParam::Request>();

        while (!map_param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "map param service not available, waiting again...");
        }

        auto result = map_param_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();

            RCLCPP_INFO(node_->get_logger(), "map param request successful");

            // Convert response to MapParam struct and store it
            map_param.map_origin_ = Eigen::Vector3d(response->map_origin.x, response->map_origin.y, response->map_origin.z);
            map_param.map_size_ = Eigen::Vector3d(response->map_size.x, response->map_size.y, response->map_size.z);
            map_param.map_min_boundary_ = Eigen::Vector3d(response->map_min_boundary.x, response->map_min_boundary.y, response->map_min_boundary.z);
            map_param.map_max_boundary_ = Eigen::Vector3d(response->map_max_boundary.x, response->map_max_boundary.y, response->map_max_boundary.z);

            map_param.map_voxel_num_ = Eigen::Vector3i(response->map_voxel_num[0], response->map_voxel_num[1], response->map_voxel_num[2]);
            map_param.resolution_ = response->resolution;
            map_param.resolution_inv_ = response->resolution_inv;
            map_param.obstacles_inflation_ = response->obstacles_inflation;
            map_param.virtual_ceil_height_ = response->virtual_ceil_height;
            map_param.ground_height_ = response->ground_height;

            map_param.box_min_ = Eigen::Vector3i(response->box_min[0], response->box_min[1], response->box_min[2]);
            map_param.box_max_ = Eigen::Vector3i(response->box_max[0], response->box_max[1], response->box_max[2]);
            map_param.box_mind_ = Eigen::Vector3d(response->box_mind.x, response->box_mind.y, response->box_mind.z);
            map_param.box_maxd_ = Eigen::Vector3d(response->box_maxd.x, response->box_maxd.y, response->box_maxd.z);

            map_param.default_dist_ = response->default_dist;
            map_param.optimistic_ = response->optimistic;
            map_param.signed_dist_ = response->signed_dist;

            // Map fusion
            map_param.p_hit_ = response->p_hit;
            map_param.p_miss_ = response->p_miss;
            map_param.p_min_ = response->p_min;
            map_param.p_max_ = response->p_max;
            map_param.p_occ_ = response->p_occ;

            // Logit
            map_param.prob_hit_log_ = response->prob_hit_log;
            map_param.prob_miss_log_ = response->prob_miss_log;
            map_param.clamp_min_log_ = response->clamp_min_log;
            map_param.clamp_max_log_ = response->clamp_max_log;
            map_param.min_occupancy_log_ = response->min_occupancy_log;

            map_param.max_ray_length_ = response->max_ray_length;
            map_param.local_bound_inflate_ = response->local_bound_inflate;
            map_param.local_map_margin_ = response->local_map_margin;
            map_param.unknown_flag_ = response->unknown_flag;

            // store the map_param in the voxel_map_ptr
            voxel_map_ptr->map_params_ = map_param;

            RCLCPP_INFO(node_->get_logger(), "Origin: (%f, %f, %f), Size: (%f, %f, %f), Voxel Num: (%d, %d, %d)",
                        voxel_map_ptr->map_params_.map_origin_.x(),
                        voxel_map_ptr->map_params_.map_origin_.y(),
                        voxel_map_ptr->map_params_.map_origin_.z(),
                        voxel_map_ptr->map_params_.map_size_.x(),
                        voxel_map_ptr->map_params_.map_size_.y(),
                        voxel_map_ptr->map_params_.map_size_.z(),
                        voxel_map_ptr->map_params_.map_voxel_num_(0),
                        voxel_map_ptr->map_params_.map_voxel_num_(1),
                        voxel_map_ptr->map_params_.map_voxel_num_(2));
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Map param service call failed");
            return false;
        }
        return true;
    }

}; // namespace ctrl_manager
