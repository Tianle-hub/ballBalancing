#ifndef PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_KALMAN
#define PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_KALMAN

#include <Eigen/Dense>
#include <ros/ros.h>

namespace BallControl
{
  class KalmanFilter
  {
    private:
    Eigen::Matrix<double,8,1> x_;
    Eigen::Matrix<double,8,8> F_;

    Eigen::Matrix<double,2,8> Hx_;
    Eigen::Matrix<double,2,8> Ha_;

    Eigen::Matrix2d Rx_;
    Eigen::Matrix2d Ra_;

    Eigen::Matrix<double,8,8> Q_;
    Eigen::Matrix<double,8,8> P_;
    
    Eigen::Matrix<double,8,2> Kx_;
    Eigen::Matrix<double,8,2> Ka_;

    Eigen::Matrix<double,8,8> I_;

    public:
    KalmanFilter();
    ~KalmanFilter();

    bool init(const Eigen::Matrix<double,8,1> &x0);

    bool updateAcc(const Eigen::Vector2d za);

    bool updatePos(const Eigen::Vector2d zx);

    bool predict();

    Eigen::Vector4d getPosVel();

    Eigen::Matrix<double,8,1> getState();
  };
}

#endif //PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_KALMAN