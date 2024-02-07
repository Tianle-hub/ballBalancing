#include <ball_controller/ball_controller.h>

namespace BallControl
{
  BallController::BallController()
  {

  }

  BallController::~BallController()
  {

  }

  bool BallController::initModel(const Eigen::Vector4d &x, const Eigen::Vector2d &input)
  {
    ROS_WARN_STREAM("BallController::init");
    std::vector<double> vec;

    // check namespace
    std::string ns = "~ball_controller";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("BallController init(): Model parameters not defined in:" << ns);
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

    // B
    ros::param::get(ns + "/K", vec);
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("K : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      K_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("K: \n" << K_);

    // state
    x_ = x;
    // input
    u_ = input;
    t_prev = 0;

    return true;
  }

  void BallController::setZero()
  {
    x_ = Eigen::Vector4d::Zero();
    u_ = Eigen::Vector2d::Zero();
  }

  void BallController::integrate(const double &timeStep)
  {
    Eigen::Vector4d xp = A_ * x_ + B_ * Eigen::Vector2d(sin(u_(0)), sin(u_(1)));
    x_ += xp * timeStep;
  }

  Eigen::Vector2d BallController::update(const Eigen::Vector4d &x)
  {
    return -K_ * x;
  }

  Eigen::Vector4d BallController::updateModel(const double &time, const Eigen::Vector2d &input)
  {
    double timeStep = time - t_prev;
    t_prev = time;

    u_ = input;
    
    integrate(timeStep);

    return x_;
  }
}