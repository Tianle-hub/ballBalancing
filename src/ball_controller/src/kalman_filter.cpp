#include <ball_controller/kalman_filter.h>

namespace BallControl
{
  KalmanFilter::KalmanFilter()
  {

  }

  KalmanFilter::~KalmanFilter()
  {

  }

  bool KalmanFilter::init(const Eigen::Matrix<double,8,1> &x0)
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
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("Hx : wrong number of dimensions:" << vec.size());
      return false;
    }
    for (size_t i = 0; i < 8; i++)
    {
      Hx_(i) = vec[i];
    }
    ROS_WARN_STREAM("Hx: \n" << Hx_);

    // Ha
    ros::param::get(ns + "/Ha", vec);
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("Ha : wrong number of dimensions:" << vec.size());
      return false;
    }
    for (size_t i = 0; i < 8; i++)
    {
      Ha_(i) = vec[i];
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
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      P_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("P: \n" << P_);

    K_ = Eigen::Matrix<double,8,8>::Zero();

    x_ = x0;

  }

}