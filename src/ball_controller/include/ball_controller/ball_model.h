#ifndef PINGPONG_BALL_MODEL_BALL_MODEL
#define PINGPONG_BALL_MODEL_BALL_MODEL

#include <Eigen/Dense>
#include <ros/ros.h>

namespace BallControl
{
  class BallModel
  {
    private:
      Eigen::Vector4d x_; // px vx py vy
      Eigen::Vector2d u_;

      Eigen::Matrix4d A_;
      Eigen::Matrix<double,4,2> B_;

      double t_prev_;

    public:
      BallModel();

      ~BallModel();

      bool initModel(const Eigen::Vector4d &x);
      
      void setZero();

      Eigen::Vector4d updateModel(const double &time, const Eigen::Vector2d &input);
    
    private:
      void integrate(const double &timeStep);
  };
}

#endif //PINGPONG_BALL_MODEL_BALL_MODEL