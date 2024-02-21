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
        break;
      
      case CAMERA:
        KalmanFilter_.init(Eigen::Matrix<double,8,1>::Zero());
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

    u_d_ = -K_ * x_;
    return u_d_;
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

  void BallController::ball_pos_vel_Callback(const ball_controller::PosVel2D ball_pos_vel)
  {
    if (ball_pos_vel.measure == true)
    {
      KalmanFilter_.updatePos(Eigen::Vector2d::Zero());
    }
    ball_pos_x = ball_pos_vel.position.x;
    ball_pos_y = ball_pos_vel.position.y;
    ball_velo_x = ball_pos_vel.velocity.linear.x;
    ball_velo_y = ball_pos_vel.velocity.linear.y;
    measure = ball_pos_vel.measure;
    ROS_INFO_STREAM("Measurement: " << measure);
    ROS_INFO("I heard: Position - [%f, %f], Velocity - [%f, %f]", ball_pos_x, ball_pos_y, ball_velo_x, ball_velo_y);
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