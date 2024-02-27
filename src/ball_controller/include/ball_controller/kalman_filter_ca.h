#ifndef PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CA_KALMAN
#define PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CA_KALMAN

#include <Eigen/Dense>
#include <ros/ros.h>

namespace BallControl
{
  class KalmanFilterCA
  {
    private:
    bool initialized_;

    Eigen::Matrix<double,6,1> x_;
    Eigen::Matrix<double,6,6> F_;

    Eigen::Matrix<double,2,6> Hx_;
    Eigen::Matrix<double,2,6> Ha_;

    Eigen::Matrix2d Rx_;
    Eigen::Matrix2d Ra_;

    Eigen::Matrix<double,6,6> Q_;
    Eigen::Matrix<double,6,6> P_;
    
    Eigen::Matrix<double,6,2> Kx_;
    Eigen::Matrix<double,6,2> Ka_;

    Eigen::Matrix<double,6,6> I_;

    public:
    KalmanFilterCA();
    ~KalmanFilterCA();

    bool gerParam();

    bool init(const Eigen::Matrix<double,6,1> &x0);

    bool updateAcc(const Eigen::Vector2d &za);

    bool updatePos(const Eigen::Vector2d &zx);

    bool predict();

    bool isInitialized();

    Eigen::Vector4d getPosVel();

    Eigen::Matrix<double,6,1> getState();
  };
}

#endif //PINGPONG_BALL_CONTROLLER_KALMAN_FILTER_CA_KALMAN