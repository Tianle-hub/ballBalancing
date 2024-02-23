#include <ball_controller/kalman_filter.h>

namespace BallControl
{
  KalmanFilter::KalmanFilter()
  {

  }

  KalmanFilter::~KalmanFilter()
  {

  }

  bool KalmanFilter::gerParam()
  {
    ROS_WARN_STREAM("KalmanFilter::init");
    std::vector<double> vec;

    // check namespace
    std::string ns = "~kalman_filter";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("KalmanFilter init(): Kalman filter parameters not defined in:" << ns);
      return false;
    }

    // F
    ros::param::get(ns + "/F", vec);
    if (vec.size() < 64)
    {
      ROS_ERROR_STREAM("F : wrong number of dimensions:" << vec.size());
      return false;
    }
    size_t k = 0;
    for (size_t i = 0; i < 8; i++)
    {
      for (size_t j = 0; j < 8; j++)
      {
      F_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("F: \n" << F_);

    // Hx
    ros::param::get(ns + "/Hx", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("Hx : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 8; j++)
      {
        Hx_(i, j) = vec[k];
        k++;
      }
    }
    ROS_WARN_STREAM("Hx: \n" << Hx_);

    // Ha
    ros::param::get(ns + "/Ha", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("Ha : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 8; j++)
      {
        Ha_(i, j) = vec[k];
        k++;
      }
    }
    ROS_WARN_STREAM("Ha: \n" << Ha_);

    // Rx
    ros::param::get(ns + "/Rx", vec);
    if (vec.size() < 4)
    {
      ROS_ERROR_STREAM("Rx : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 2; j++)
      {
      Rx_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("Rx: \n" << Rx_);

    // Ra
    ros::param::get(ns + "/Ra", vec);
    if (vec.size() < 4)
    {
      ROS_ERROR_STREAM("Ra : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 2; j++)
      {
      Ra_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("Ra: \n" << Ra_);

    // Q
    ros::param::get(ns + "/Q", vec);
    if (vec.size() < 64)
    {
      ROS_ERROR_STREAM("Q : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 8; i++)
    {
      for (size_t j = 0; j < 8; j++)
      {
      Q_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("Q: \n" << Q_);

    // P
    ros::param::get(ns + "/P", vec);
    if (vec.size() < 64)
    {
      ROS_ERROR_STREAM("P : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 8; i++)
    {
      for (size_t j = 0; j < 8; j++)
      {
      P_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("P: \n" << P_);

    Kx_ = Eigen::Matrix<double,8,2>::Zero();
    Ka_ = Eigen::Matrix<double,8,2>::Zero();

    I_ = Eigen::Matrix<double,8,8>::Identity();

    x_ = Eigen::Matrix<double,8,1>::Zero();

    initialized_ = false;

    return true;
  }

  bool KalmanFilter::init(const Eigen::Matrix<double,8,1> &x0)
  {
    x_ = x0;
    initialized_ = true;
    return initialized_;
  }

  bool KalmanFilter::predict()
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Kalman filter is not initialized!");
      return false;
    } else
    {
      x_ = F_ * x_;
      P_ = F_ * P_ * F_.transpose() + Q_;
      return true;
    }
  }

  bool KalmanFilter::updatePos(const Eigen::Vector2d &zx)
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Kalman filter is not initialized!");
      return false;
    } else
    {
      Kx_ = P_ * Hx_.transpose() * (Hx_ * P_ * Hx_.transpose() + Rx_.transpose()).inverse();
      x_ = (I_ - Kx_ * Hx_) * x_ + Kx_ * zx;
      P_ = (I_ - Kx_ * Hx_) * P_;
      ROS_INFO_STREAM("updatePos_x_" << x_.transpose());
      return true;
    }
  }

  bool KalmanFilter::updateAcc(const Eigen::Vector2d &za)
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Kalman filter is not initialized!");
      return false;
    } else
    {
      Ka_ = P_ * Ha_.transpose() * (Ha_ * P_ * Ha_.transpose() + Ra_.transpose()).inverse();
      x_ = (I_ - Ka_ * Ha_) * x_ + Ka_ * za;
      ROS_INFO_STREAM("updateAcc_x_" << x_.transpose());
      P_ = (I_ - Ka_ * Ha_) * P_;
      return true;
      }
  }

  Eigen::Matrix<double,8,1> KalmanFilter::getState()
  {
    return x_;
  }

  Eigen::Vector4d KalmanFilter::getPosVel()
  {
    return Eigen::Vector4d(x_(0), x_(1), x_(4), x_(5));
  }

  bool KalmanFilter::isInitialized()
  {
    return initialized_;
  }
}