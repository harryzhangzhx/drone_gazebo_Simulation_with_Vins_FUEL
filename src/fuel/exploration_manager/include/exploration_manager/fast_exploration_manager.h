#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class EDTEnvironment;
  class SDFMap;
  class FastPlannerManager;
  class FrontierFinder;
  struct ExplorationParam;
  struct ExplorationData;

  enum EXPL_RESULT
  {
    NO_FRONTIER,
    FAIL,
    SUCCEED
  };

  class FastExplorationManager
  {
  public:
    FastExplorationManager();
    ~FastExplorationManager();

    void initialize(const std::shared_ptr<rclcpp::Node> &node);

    int planExploreMotion(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                          const Vector3d &yaw);

    // Benchmark method, classic frontier and rapid frontier
    int classicFrontier(const Vector3d &pos, const double &yaw);
    int rapidFrontier(const Vector3d &pos, const Vector3d &vel, const double &yaw, bool &classic);

    shared_ptr<ExplorationData> ed_;
    shared_ptr<ExplorationParam> ep_;
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FrontierFinder> frontier_finder_;
    // unique_ptr<ViewFinder> view_finder_;
    // ros::Publisher endpose_pub_;
    // ros::Publisher cmu_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr endpose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cmu_pose_pub_;

    geometry_msgs::msg::Point end_pose;
    geometry_msgs::msg::PointStamped cmu_pose;
    bool cmu_exploration;

  private:
    shared_ptr<EDTEnvironment> edt_environment_;
    shared_ptr<SDFMap> sdf_map_;

    // Find optimal tour for coarse viewpoints of all frontiers
    void findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                        vector<int> &indices);

    // Refine local tour for next few frontiers, using more diverse viewpoints
    void refineLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                         const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
                         vector<Vector3d> &refined_pts, vector<double> &refined_yaws);

    void shortenPath(vector<Vector3d> &path);

  public:
    typedef shared_ptr<FastExplorationManager> Ptr;
  };

} // namespace fast_planner

#endif