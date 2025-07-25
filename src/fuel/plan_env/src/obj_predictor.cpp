#include <plan_env/obj_predictor.h>
#include <string>
#include <memory>
#include <cmath>

namespace fast_planner
{

  int ObjHistory::queue_size_;
  int ObjHistory::skip_num_;
  rclcpp::Time ObjHistory::global_start_time_;

  void ObjHistory::init(int id)
  {
    clear();
    skip_ = 0;
    obj_idx_ = id;
  }

  void ObjHistory::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    ++skip_;
    if (skip_ < ObjHistory::skip_num_)
      return;

    Eigen::Vector4d pos_t;
    pos_t(0) = msg->pose.position.x;
    pos_t(1) = msg->pose.position.y;
    pos_t(2) = msg->pose.position.z;
    pos_t(3) = (rclcpp::Clock(RCL_ROS_TIME).now() - ObjHistory::global_start_time_).seconds();

    history_.push_back(pos_t);

    if (history_.size() > static_cast<size_t>(queue_size_))
      history_.pop_front();

    skip_ = 0;
  }

  ObjPredictor::ObjPredictor() {}

  ObjPredictor::ObjPredictor(rclcpp::Node::SharedPtr node)
  {
    this->node_handle_ = node;
  }

  ObjPredictor::~ObjPredictor() {}

  void ObjPredictor::init()
  {
    // node_handle_->declare_parameter("prediction/obj_num", 5);
    // node_handle_->declare_parameter("prediction/lambda", 1.0);
    // node_handle_->declare_parameter("prediction/predict_rate", 1.0);

    node_handle_->get_parameter_or("prediction/obj_num", obj_num_, 5);
    node_handle_->get_parameter_or("prediction/lambda", lambda_, 1.0);
    node_handle_->get_parameter_or("prediction/predict_rate", predict_rate_, 1.0);

    predict_trajs_ = std::make_shared<vector<PolynomialPrediction>>();
    predict_trajs_->resize(obj_num_);

    obj_scale_ = std::make_shared<vector<Eigen::Vector3d>>();
    obj_scale_->resize(obj_num_);
    scale_init_.resize(obj_num_, false);

    for (int i = 0; i < obj_num_; i++)
    {
      auto obj_his = std::make_shared<ObjHistory>();
      obj_his->init(i);
      obj_histories_.push_back(obj_his);

      auto sub = node_handle_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/dynamic/pose_" + std::to_string(i), 10,
          [obj_his](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
          { obj_his->poseCallback(msg); });

      pose_subs_.push_back(sub);
    }

    marker_sub_ = node_handle_->create_subscription<visualization_msgs::msg::Marker>(
        "/dynamic/obj", 10,
        std::bind(&ObjPredictor::markerCallback, this, std::placeholders::_1));

    predict_timer_ = node_handle_->create_wall_timer(
        std::chrono::duration<double>(1.0 / predict_rate_),
        std::bind(&ObjPredictor::predictCallback, this));
  }

  ObjPrediction ObjPredictor::getPredictionTraj()
  {
    return this->predict_trajs_;
  }

  ObjScale ObjPredictor::getObjScale()
  {
    return this->obj_scale_;
  }

  void ObjPredictor::predictPolyFit()
  {
    for (int i = 0; i < obj_num_; i++)
    {
      Eigen::Matrix<double, 6, 6> A;
      Eigen::Matrix<double, 6, 1> temp;
      Eigen::Matrix<double, 6, 1> bm[3];
      vector<Eigen::Matrix<double, 6, 1>> pm(3);

      A.setZero();
      for (int j = 0; j < 3; ++j)
        bm[j].setZero();

      list<Eigen::Vector4d> his;
      obj_histories_[i]->getHistory(his);
      for (auto it = his.begin(); it != his.end(); ++it)
      {
        Eigen::Vector3d qi = (*it).head(3);
        double ti = (*it)(3);

        temp << 1.0, ti, pow(ti, 2), pow(ti, 3), pow(ti, 4), pow(ti, 5);
        for (int j = 0; j < 6; ++j)
          A.row(j) += 2.0 * pow(ti, j) * temp.transpose();

        for (int dim = 0; dim < 3; ++dim)
          bm[dim] += 2.0 * qi(dim) * temp;
      }

      if (his.empty())
        continue;

      double t1 = his.front()(3);
      double t2 = his.back()(3);

      temp << 0.0, 0.0, 2 * t1 - 2 * t2, 3 * pow(t1, 2) - 3 * pow(t2, 2), 4 * pow(t1, 3) - 4 * pow(t2, 3),
          5 * pow(t1, 4) - 5 * pow(t2, 4);
      A.row(2) += -4 * lambda_ * temp.transpose();

      temp << 0.0, 0.0, pow(t1, 2) - pow(t2, 2), 2 * pow(t1, 3) - 2 * pow(t2, 3),
          3 * pow(t1, 4) - 3 * pow(t2, 4), 4 * pow(t1, 5) - 4 * pow(t2, 5);
      A.row(3) += -12 * lambda_ * temp.transpose();

      temp << 0.0, 0.0, 20 * pow(t1, 3) - 20 * pow(t2, 3), 45 * pow(t1, 4) - 45 * pow(t2, 4),
          72 * pow(t1, 5) - 72 * pow(t2, 5), 100 * pow(t1, 6) - 100 * pow(t2, 6);
      A.row(4) += -4.0 / 5.0 * lambda_ * temp.transpose();

      temp << 0.0, 0.0, 35 * pow(t1, 4) - 35 * pow(t2, 4), 84 * pow(t1, 5) - 84 * pow(t2, 5),
          140 * pow(t1, 6) - 140 * pow(t2, 6), 200 * pow(t1, 7) - 200 * pow(t2, 7);
      A.row(5) += -4.0 / 7.0 * lambda_ * temp.transpose();

      for (int j = 0; j < 3; j++)
      {
        pm[j] = A.colPivHouseholderQr().solve(bm[j]);
      }

      predict_trajs_->at(i).setPolynomial(pm);
      predict_trajs_->at(i).setTime(t1, t2);
    }
  }

  void ObjPredictor::predictCallback()
  {
    // predictPolyFit();
    predictConstVel();
  }

  void ObjPredictor::markerCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg)
  {
    int idx = msg->id;
    (*obj_scale_)[idx](0) = msg->scale.x;
    (*obj_scale_)[idx](1) = msg->scale.y;
    (*obj_scale_)[idx](2) = msg->scale.z;

    scale_init_[idx] = true;

    int finish_num = 0;
    for (int i = 0; i < obj_num_; i++)
    {
      if (scale_init_[i])
        finish_num++;
    }

    if (finish_num == obj_num_)
    {
      marker_sub_.reset();
    }
  }

  void ObjPredictor::predictConstVel()
  {
    for (int i = 0; i < obj_num_; i++)
    {
      list<Eigen::Vector4d> his;
      obj_histories_[i]->getHistory(his);
      if (his.size() < 2)
        continue;
      auto list_it = his.end();

      Eigen::Vector3d q1, q2;
      double t1, t2;

      --list_it;
      q2 = (*list_it).head(3);
      t2 = (*list_it)(3);

      --list_it;
      q1 = (*list_it).head(3);
      t1 = (*list_it)(3);

      Eigen::Matrix<double, 2, 3> q12;
      q12.row(0) = q1.transpose();
      q12.row(1) = q2.transpose();

      Eigen::Matrix<double, 2, 2> At12;
      At12 << 1, t1, 1, t2;

      Eigen::Matrix<double, 2, 3> p01 = At12.inverse() * q12;

      vector<Eigen::Matrix<double, 6, 1>> polys(3);
      for (int j = 0; j < 3; ++j)
      {
        polys[j].setZero();
        polys[j].head(2) = p01.col(j);
      }

      predict_trajs_->at(i).setPolynomial(polys);
      predict_trajs_->at(i).setTime(t1, t2);
    }
  }

} // namespace fast_planner