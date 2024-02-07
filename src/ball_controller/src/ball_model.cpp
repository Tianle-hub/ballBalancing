#include <ball_controller/ball_model.h>

namespace BallController
{
  BallModel::BallModel()
  {

  }

  BallModel::~BallModel()
  {

  }

  bool BallModel::initModel(const Eigen::Vector4d &x, const Eigen::Vector2d &input)
  {
    ROS_WARN_STREAM("BallModel::init");
    std::vector<double> vec;

    // check namespace
    std::string ns = "~ball_model";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("BallModel init(): Model parameters not defined in:" << ns);
      return false;
    }

    // A
    ros::param::get(ns + "/A", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("A : wrong number of dimensions:" << vec.size());
      return false;
    }
    size_t k = 0;
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      A_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("A: \n" << A_);

    // B
    ros::param::get(ns + "/B", vec);
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("B : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 2; j++)
      {
      B_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("B: \n" << B_);

    // state
    x_ = x;
    // input
    u_ = input;
    t_prev = 0;

    return true;
  }

  void BallModel::setZero()
  {
    x_ = Eigen::Vector4d::Zero();
    u_ = Eigen::Vector2d::Zero();
  }

  void BallModel::integrate(const double &timeStep)
  {
    Eigen::Vector4d xp = A_ * x_ + B_ * Eigen::Vector2d(sin(u_(0)), sin(u_(1)));
    x_ += xp * timeStep;
  }

  Eigen::Vector4d BallModel::update(const double &time, const Eigen::Vector2d &input)
  {
    double timeStep = time - t_prev;
    t_prev = time;

    u_ = input;
    
    integrate(timeStep);

    return x_;
  }
}