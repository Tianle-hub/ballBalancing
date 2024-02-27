#include <ball_controller/kalman_filter_cv.h>

namespace BallControl
{
  KalmanFilterCV::KalmanFilterCV()
  {

  }

  KalmanFilterCV::~KalmanFilterCV()
  {

  }

  bool KalmanFilterCV::gerParam()
  {
    ROS_WARN_STREAM("KalmanFilterCV::init");
    std::vector<double> vec;
    double var;

    // check namespace
    std::string ns = "~kalman_filter_cv";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("KalmanFilter init(): Kalman filter parameters not defined in:" << ns);
      return false;
    }

    // F
    ros::param::get(ns + "/F", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("F : wrong number of dimensions:" << vec.size());
      return false;
    }
    size_t k = 0;
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      F_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("F: \n" << F_);

    // Hx
    ros::param::get(ns + "/Hx", vec);
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("Hx : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        Hx_(i, j) = vec[k];
        k++;
      }
    }
    ROS_WARN_STREAM("Hx: \n" << Hx_);


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


    // Q
    ros::param::get(ns + "/Q", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("Q : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      Q_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("Q: \n" << Q_);

    // P
    ros::param::get(ns + "/P", vec);
    if (vec.size() < 16)
    {
      ROS_ERROR_STREAM("P : wrong number of dimensions:" << vec.size());
      return false;
    }
    k = 0;
    for (size_t i = 0; i < 4; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      P_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("P: \n" << P_);

    Kx_ = Eigen::Matrix<double,4,2>::Zero();

    I_ = Eigen::Matrix<double,4,4>::Identity();

    x_ = Eigen::Matrix<double,4,1>::Zero();

    initialized_ = false;

    return true;
  }

  bool KalmanFilterCV::init(const Eigen::Matrix<double,4,1> &x0)
  {
    x_ = x0;
    initialized_ = true;
    return initialized_;
  }

  bool KalmanFilterCV::predict()
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

  bool KalmanFilterCV::updatePos(const Eigen::Vector2d &zx)
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
      return true;
    }
  }


  Eigen::Vector4d KalmanFilterCV::getPosVel()
  {
    return x_;
  }

  bool KalmanFilterCV::isInitialized()
  {
    return initialized_;
  }
}