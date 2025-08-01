#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <vector>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class PolynomialPrediction;
  typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
  typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

  /* ========== prediction polynomial ========== */
  class PolynomialPrediction
  {
  private:
    vector<Eigen::Matrix<double, 6, 1>> polys;
    double t1, t2; // start / end

  public:
    PolynomialPrediction() {}
    ~PolynomialPrediction() {}

    void setPolynomial(const vector<Eigen::Matrix<double, 6, 1>> &pls)
    {
      polys = pls;
    }
    void setTime(double t1, double t2)
    {
      this->t1 = t1;
      this->t2 = t2;
    }

    bool valid()
    {
      return polys.size() == 3;
    }

    /* note that t should be in [t1, t2] */
    Eigen::Vector3d evaluate(double t)
    {
      Eigen::Matrix<double, 6, 1> tv;
      tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

      return pt;
    }

    Eigen::Vector3d evaluateConstVel(double t)
    {
      Eigen::Matrix<double, 2, 1> tv;
      tv << 1.0, pow(t, 1);

      Eigen::Vector3d pt;
      pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

      return pt;
    }
  };

  /* ========== subscribe and record object history ========== */
  class ObjHistory
  {
  public:
    static int skip_num_;
    static int queue_size_;
    static rclcpp::Time global_start_time_;

    ObjHistory() {}
    ~ObjHistory() {}

    void init(int id);

    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void clear()
    {
      history_.clear();
    }

    void getHistory(list<Eigen::Vector4d> &his)
    {
      his = history_;
    }

  private:
    list<Eigen::Vector4d> history_; // x,y,z;t
    int skip_;
    int obj_idx_;
    Eigen::Vector3d scale_;
  };

  /* ========== predict future trajectory using history ========== */
  class ObjPredictor
  {
  private:
    rclcpp::Node::SharedPtr node_handle_;

    int obj_num_;
    double lambda_;
    double predict_rate_;

    vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    rclcpp::TimerBase::SharedPtr predict_timer_;
    vector<shared_ptr<ObjHistory>> obj_histories_;

    /* share data with planner */
    ObjPrediction predict_trajs_;
    ObjScale obj_scale_;
    vector<bool> scale_init_;

    void markerCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg);

    void predictCallback();
    void predictPolyFit();
    void predictConstVel();

  public:
    ObjPredictor();
    ObjPredictor(rclcpp::Node::SharedPtr node);
    ~ObjPredictor();

    void init();

    ObjPrediction getPredictionTraj();
    ObjScale getObjScale();

    typedef shared_ptr<ObjPredictor> Ptr;
  };

} // namespace fast_planner

#endif