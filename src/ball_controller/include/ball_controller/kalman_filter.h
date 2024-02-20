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

    Eigen::Matrix<double,1,8> Hx_;
    Eigen::Matrix<double,8,1> Ha_;

    Eigen::Matrix2d Rx_;
    Eigen::Matrix2d Ra_;

    Eigen::Matrix<double,8,8> Q_;
    Eigen::Matrix<double,8,8> P_;
    
    Eigen::Matrix<double,8,8> K_;

    public:
    KalmanFilter();
    ~KalmanFilter();

    bool init(const Eigen::Matrix<double,8,1> &x0);

    bool updateAcc();

    bool updatePos();



  }
}

#endif //PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_KALMAN