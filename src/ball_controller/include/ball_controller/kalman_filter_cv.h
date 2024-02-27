#ifndef PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CV_KALMAN
#define PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CV_KALMAN

#include <Eigen/Dense>
#include <ros/ros.h>

namespace BallControl
{
  class KalmanFilterCV
  {
    private:
    bool initialized_;

    Eigen::Matrix<double,4,1> x_;
    Eigen::Matrix<double,4,4> F_;

    Eigen::Matrix<double,2,4> Hx_;

    Eigen::Matrix2d Rx_;

    Eigen::Matrix<double,4,4> Q_;
    Eigen::Matrix<double,4,4> P_;
    
    Eigen::Matrix<double,4,2> Kx_;

    Eigen::Matrix<double,4,4> I_;

    public:
    KalmanFilterCV();
    ~KalmanFilterCV();

    bool gerParam();

    bool init(const Eigen::Matrix<double,4,1> &x0);

    bool updatePos(const Eigen::Vector2d &zx);

    bool predict();

    bool isInitialized();

    Eigen::Vector4d getPosVel();

  };
}

#endif //PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CV_KALMAN