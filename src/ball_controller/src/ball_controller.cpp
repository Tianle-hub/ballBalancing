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

    unmeasured_frames_ = 0;

    initK();

    position_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_position", 1);
    velocity_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_velocity", 1);
    acceleration_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_acceleration_kalman", 1);
    angel_plate_pb_  = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_acceleration_plate", 1);
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

    position_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_position_kalman", 1);
    velocity_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_velocity_kalman", 1);
    acceleration_pb_ = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_acceleration_kalman", 1);
    angel_plate_pb_  = nh_.advertise<geometry_msgs::Vector3Stamped>("ball_acceleration_plate", 1);
    camera_sub_ = nh_.subscribe("ball_pos_vel", 1000, &BallController::ball_pos_vel_Callback, this);

    return true;
  }

  Eigen::Vector2d BallController::update(const double &time, const Eigen::Vector2d &u)
  {
    switch(controllerState_)
    {
      case WAITING:
        // ROS_INFO_STREAM("waitin_x_:" << Eigen::Vector2d::Zero().transpose());
        return Eigen::Vector2d::Zero();
        break;

      case RUNNING:
        switch(ballType_)
        {
          case MODEL:
            x_ = ballModel_.updateModel(time, u);
            // ROS_INFO_STREAM("model_x_:" << x_.transpose()); 

            break;
          
          case CAMERA:
            KalmanFilter_.predict();
            KalmanFilter_.updateAcc(Eigen::Vector2d(sin(u(0)), sin(u(1))));
            x_ = KalmanFilter_.getPosVel();
            // ROS_INFO_STREAM("camera_x_:" << x_.transpose());  Eigen::Vector2d::Zero()
            break;
        }

        pubPlateAngel(u);
        pubBallTF();
        pubState();

        u_d_ = -1*K_ * x_; //0.7
        return u_d_;

        break;

      default:
        // ROS_INFO_STREAM("default_x_:" << Eigen::Vector2d::Zero().transpose());
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

  void BallController::pubPlateAngel(const Eigen::Vector2d &u)
  {
    geometry_msgs::Vector3Stamped ball_acceleration_plate;
    ball_acceleration_plate.header.stamp = ros::Time::now();

    ball_acceleration_plate.vector.x = u(0);
    ball_acceleration_plate.vector.y = u(1);

    angel_plate_pb_.publish(ball_acceleration_plate);
  }

  void BallController::pubState()
  {
      Eigen::Matrix<double,8,1> x = KalmanFilter_.getState();
      geometry_msgs::Vector3Stamped ball_position;
      geometry_msgs::Vector3Stamped ball_velocity;
      geometry_msgs::Vector3Stamped ball_acceleration;

      ball_position.header.stamp = ros::Time::now();
      ball_velocity.header.stamp = ros::Time::now();
      ball_acceleration.header.stamp = ros::Time::now();

      ball_position.vector.x = x(0);
      ball_position.vector.y = x(4);

      ball_velocity.vector.x = x(1);
      ball_velocity.vector.y = x(5);

      ball_acceleration.vector.x = x(2);
      ball_acceleration.vector.y = x(6);

      position_pb_.publish(ball_position);
      velocity_pb_.publish(ball_velocity);
      acceleration_pb_.publish(ball_acceleration);
  }

  void BallController::ball_pos_vel_Callback(const ball_controller::PosVel2D ball_pos_vel)
  {
    if (ball_pos_vel.measure == true)
    {
      ball_pos_x = ball_pos_vel.position.x;
      ball_pos_y = ball_pos_vel.position.y;

      unmeasured_frames_ = 0;

      if (!KalmanFilter_.isInitialized())
      {
        Eigen::Matrix<double,8,1> x0 = Eigen::Matrix<double,8,1>::Zero();
        x0(0) = ball_pos_x;
        x0(4) = ball_pos_y;

        if (KalmanFilter_.init(x0))
        {
          ROS_INFO_STREAM("Successfully initialize kalman filter with state: " << x0.transpose());
        } else
        {
          ROS_WARN_STREAM("Falied to initialize kalman filter!");
        }

      } else
      {
        Eigen::Vector2d pos(ball_pos_x, ball_pos_y);
        KalmanFilter_.updatePos(pos);
        
        // ROS_INFO_STREAM("Measurement: " << measure);
        // ROS_INFO("I heard: Position - [%f, %f]", ball_pos_x, ball_pos_y);
      }
    } else
    {
      if (KalmanFilter_.isInitialized())
      {
        unmeasured_frames_ ++;
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