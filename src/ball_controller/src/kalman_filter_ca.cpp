#include <ball_controller/kalman_filter_ca.h>

namespace BallControl
{
  KalmanFilterCA::KalmanFilterCA()
  {

  }

  KalmanFilterCA::~KalmanFilterCA()
  {

  }

  bool KalmanFilterCA::gerParam()
  {
    ROS_WARN_STREAM("KalmanFilterCA::init");
    std::vector<double> vec;
    double var;

    // check namespace
    std::string ns = "~kalman_filter_ca";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("KalmanFilter init(): Kalman filter parameters not defined in:" << ns);
      return false;
    }

    // F
    ros::param::get(ns + "/F", vec);
    if (vec.size() < 36)
    {
      ROS_ERROR_STREAM("F : wrong number of dimensions:" << vec.size());
      return false;
    }
    size_t k = 0;
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
      F_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("F: \n" << F_);

    // Hx
    ros::param::get(ns + "/Hx", vec);
    if (vec.size() < 12)
    {
      ROS_ERROR_STREAM("Hx : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
        Hx_(i, j) = vec[k];
        k++;
      }
    }
    ROS_WARN_STREAM("Hx: \n" << Hx_);

    // Ha
    ros::param::get(ns + "/Ha", vec);
    if (vec.size() < 12)
    {
      ROS_ERROR_STREAM("Ha : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 6; j++)
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
    if (vec.size() < 36)
    {
      ROS_ERROR_STREAM("Q : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
      Q_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("Q: \n" << Q_);

    // P
    ros::param::get(ns + "/P", vec);
    if (vec.size() < 36)
    {
      ROS_ERROR_STREAM("P : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
      P_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("P: \n" << P_);

    Kx_ = Eigen::Matrix<double,6,2>::Zero();
    Ka_ = Eigen::Matrix<double,6,2>::Zero();

    I_ = Eigen::Matrix<double,6,6>::Identity();

    x_ = Eigen::Matrix<double,6,1>::Zero();

    initialized_ = false;

    return true;
  }

  bool KalmanFilterCA::init(const Eigen::Matrix<double,6,1> &x0)
  {
    x_ = x0;
    initialized_ = true;
    return initialized_;
  }

  bool KalmanFilterCA::predict()
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

  bool KalmanFilterCA::updatePos(const Eigen::Vector2d &zx)
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Kalman filter is not initialized!");
      return false;
    } else
    {
      Kx_ = P_ * Hx_.transpose() * (Hx_ * P_ * Hx_.transpose() + Rx_).inverse();
      x_ = (I_ - Kx_ * Hx_) * x_ + Kx_ * zx;
      P_ = (I_ - Kx_ * Hx_) * P_;
      return true;
    }
  }

  bool KalmanFilterCA::updateAcc(const Eigen::Vector2d &za)
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Kalman filter is not initialized!");
      return false;
    } else
    {
      Ka_ = P_ * Ha_.transpose() * (Ha_ * P_ * Ha_.transpose() + Ra_).inverse();
      x_ = (I_ - Ka_ * Ha_) * x_ + Ka_ * za;
      P_ = (I_ - Ka_ * Ha_) * P_;
      return true;
      }
  }

  Eigen::Matrix<double,6,1> KalmanFilterCA::getState()
  {
    return x_;
  }

  Eigen::Vector4d KalmanFilterCA::getPosVel()
  {
    return Eigen::Vector4d(x_(0), x_(1), x_(3), x_(4));
  }

  bool KalmanFilterCA::isInitialized()
  {
    return initialized_;
  }
}