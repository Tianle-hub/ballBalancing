#ifndef PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER
#define PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "ball_controller/PosVel2D.h"

namespace BallControl
{
  class BallController
  {
    private:
      // double x_;
      // double xd_;
      // double y_;
      // double yd_;
      // double theta_;
      // double phi_;
      Eigen::Vector4d x_;
      Eigen::Vector2d u_;

      Eigen::Matrix4d A_;
      Eigen::Matrix<double,4,2> B_;

      Eigen::Matrix<double,2,4> K_;

      double t_prev_;

      ros::NodeHandle nh_;
      
      ros::Publisher position_pb_;
      ros::Publisher velocity_pb_;

      tf::TransformBroadcaster br_;

      double ball_pos_x;
      double ball_pos_y;
      double ball_velo_x;
      double ball_velo_y;

    public:
      BallController();

      ~BallController();

      bool initModel(const Eigen::Vector4d &x, const Eigen::Vector2d &input);
      
      void setZero();

      Eigen::Vector4d updateModel(const double &time, const Eigen::Vector2d &input);

      Eigen::Vector2d update(const Eigen::Vector4d &x);

      void ball_pos_vel_Callback(const ball_controller::PosVel2D);
      ros::Subscriber sub = nh_.subscribe("ball_pos_vel", 1000, &BallController::ball_pos_vel_Callback, this);
    
    private:

      void integrate(const double &timeStep);

      void pubBallTF();

  

  };
}

#endif //PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER