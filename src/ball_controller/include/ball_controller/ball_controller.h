#ifndef PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER
#define PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "ball_controller/PosVel2D.h"
#include <ball_controller/kalman_filter.h>
#include <ball_controller/ball_model.h>

namespace BallControl
{
  enum BallType
  {
    MODEL,
    CAMERA
  };

  enum ControllerState
  {
    RUNNING,
    WAITING
  };
  
  class BallController
  {
    private:
      BallType ballType_;
      ControllerState controllerState_;

      Eigen::Vector4d x_; // px vx py vy
      Eigen::Vector2d u_d_;

      Eigen::Matrix<double,2,4> K_;

      BallControl::BallModel ballModel_;

      BallControl::KalmanFilter KalmanFilter_;

      ros::NodeHandle nh_;
      
      ros::Publisher position_pb_;
      ros::Publisher velocity_pb_;

      tf::TransformBroadcaster br_;

      // why here no need to add class_name::variable_name
      double ball_pos_x;  
      double ball_pos_y;
      double ball_velo_x;
      double ball_velo_y;
      bool measure;

      ros::Subscriber camera_sub_;

    public:
      BallController();

      ~BallController();

      bool init(const Eigen::Vector4d &x, const BallControl::BallType ballType);

      bool init(const BallControl::BallType ballType);
      
      Eigen::Vector2d update(const double &time, const Eigen::Vector2d &u);

      Eigen::Vector4d getState();
    
    private:
      void pubBallTF();

      void ball_pos_vel_Callback(const ball_controller::PosVel2D);

      bool initK();

      void pubState();

  };
}

#endif //PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER