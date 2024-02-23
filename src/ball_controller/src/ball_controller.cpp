#include <ball_controller/ball_controller.h>
// #include "ball_controller/PosVel2D.h"

namespace BallControl
{
  BallController::BallController()
  {

  }

  BallController::~BallController()
  {

  }

  bool BallController::init(const Eigen::Vector4d &x, const BallType ballType)
  {
    ballType_ = ballType;

    switch(ballType_)
    {
      case MODEL:
        ballModel_.initModel(x);
        controllerState_ = ControllerState::RUNNING;
        break;
      
      case CAMERA:
        KalmanFilter_.gerParam();
        controllerState_ = ControllerState::WAITING;
        ROS_WARN_STREAM("waiting for ball...");
        break;
    }    
    // state
    x_ = x;
    // input
    u_d_ = Eigen::Vector2d::Zero();

    initK();

    position_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_position", 1);
    velocity_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_velocity", 1);
    camera_sub_ = nh_.subscribe("ball_pos_vel", 1000, &BallController::ball_pos_vel_Callback, this);

    return true;
  }

  bool BallController::init(const BallType ballType)
  {
    ballType_ = ballType;

    Eigen::Vector4d x = Eigen::Vector4d::Zero();

    switch(ballType_)
    {
      case MODEL:
        ballModel_.initModel(x);
        controllerState_ = ControllerState::RUNNING;
        break;
      
      case CAMERA:
        KalmanFilter_.gerParam();
        controllerState_ = ControllerState::WAITING;
        break;
    }    
    // state
    x_ = x;
    // input
    u_d_ = Eigen::Vector2d::Zero();

    initK();

    position_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_position", 1);
    velocity_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_velocity", 1);
    camera_sub_ = nh_.subscribe("ball_pos_vel", 1000, &BallController::ball_pos_vel_Callback, this);

    return true;
  }

  Eigen::Vector2d BallController::update(const double &time, const Eigen::Vector2d &u)
  {
    switch(controllerState_)
    {
      case WAITING:
        return Eigen::Vector2d::Zero();
        break;

      case RUNNING:
        switch(ballType_)
        {
          case MODEL:
            x_ = ballModel_.updateModel(time, u);
            break;
          
          case CAMERA:
            KalmanFilter_.predict();
            KalmanFilter_.updateAcc(u);
            x_ = KalmanFilter_.getPosVel();
            break;
        }

        pubBallTF();
        pubState();

        u_d_ = -K_ * x_;
        return u_d_;

        break;

      default:
        return Eigen::Vector2d::Zero();
        break;
    }
  }


  void BallController::pubBallTF()
  {
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(x_(0), x_(2), 0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_joint_6", "ball"));
  }

  void BallController::pubState()
  {
      geometry_msgs::Vector3Stamped ball_position;
      geometry_msgs::Vector3Stamped ball_velocity;

      ball_position.header.stamp = ros::Time::now();
      ball_velocity.header.stamp = ros::Time::now();

      ball_position.vector.x = x_(0);
      ball_position.vector.y = x_(2);

      ball_velocity.vector.x = x_(1);
      ball_velocity.vector.y = x_(3);

      position_pb_.publish(ball_position);
      velocity_pb_.publish(ball_velocity);
  }

  void BallController::ball_pos_vel_Callback(const ball_controller::PosVel2D ball_pos_vel)
  {
    if (ball_pos_vel.measure == true)
    {
      ball_pos_x = ball_pos_vel.position.x;
      ball_pos_y = ball_pos_vel.position.y;

      if (!KalmanFilter_.isInitialized())
      {
        Eigen::Matrix<double,8,1> x0 = Eigen::Matrix<double,8,1>::Zero();
        x0(0) = ball_pos_x;
        x0(4) = ball_pos_y;

        if (KalmanFilter_.init(x0))
        {
          ROS_INFO_STREAM("Successfully initialize kalman filter.");
        } else
        {
          ROS_WARN_STREAM("Falied to initialize kalman filter!");
        }

      } else
      {
        Eigen::Vector2d pos(ball_pos_x, ball_pos_y);
        KalmanFilter_.updatePos(pos);
        
        ROS_INFO_STREAM("Measurement: " << measure);
        ROS_INFO("I heard: Position - [%f, %f]", ball_pos_x, ball_pos_y);
      }
    }

  }

  Eigen::Vector4d BallController::getState()
  {
    return x_;
  }

  bool BallController::initK()
  {
    ROS_WARN_STREAM("BallController::init");
    std::vector<double> vec;  // Vector to store the retrieved values

    // check namespace
    std::string ns = "~ball_controller";
    if (!ros::param::has(ns))
    {
      ROS_ERROR_STREAM("BallController init(): LQR parameters not defined in:" << ns);
      return false;
    }
    ros::param::get(ns + "/K", vec);
    if (vec.size() < 8)
    {
      ROS_ERROR_STREAM("K : wrong number of dimensions:" << vec.size());
      return false;
    }
    size_t k = 0;
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
      K_(i, j) = vec[k];
      k++;
      }
    }
    ROS_WARN_STREAM("K: \n" << K_);

    return true;
  }
}