#ifndef PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER
#define PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>

namespace BallController 
{
  class BallModel
  {
    private:
      // double x_;
      // double xd_;
      // double y_;
      // double yd_;
      // double theta_;
      // double phi_;
      Eigen::Vector4d x_;
      Eigen::Vector2d u_;

      Eigen::Matrix4d A_;
      Eigen::Matrix<double,4,2> B_;

      double t_prev;

    public:
      BallModel();

      ~BallModel();

      bool initModel(const Eigen::Vector4d &x, const Eigen::Vector2d &input);
      
      void setZero();

      Eigen::Vector4d update(const double &time, const Eigen::Vector2d &input);

    private:

      void integrate(const double &timeStep);




  };
}

#endif //PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER