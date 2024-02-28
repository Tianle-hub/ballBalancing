#ifndef PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER
#define PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include "ball_controller/PosVel2D.h"
#include <ball_controller/kalman_filter.h>
#include <ball_controller/ball_model.h>
#include <ball_controller/ball_traj_planner.h>

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

      Eigen::Matrix2d kp_;
      Eigen::Matrix2d kd_;
      Eigen::Matrix2d ki_;

      Eigen::Vector4d x_; // px vx py vy
      Eigen::Vector4d x_prev_;
      Eigen::Vector2d u_d_;
      Eigen::Vector2d x_d_;

      Eigen::Matrix<double,2,4> K_;

      BallControl::BallModel ballModel_;

      BallControl::TrajPlanner traj_planner_;

      BallControl::KalmanFilter KalmanFilter_;

      ros::NodeHandle nh_;
      
      ros::Publisher position_pb_;
      ros::Publisher velocity_pb_;
      ros::Publisher acceleration_pb_;
      ros::Publisher angel_plate_pb_;

      ros::Subscriber camera_sub_;
      ros::Subscriber position_desired_sub_;

      tf::TransformBroadcaster br_;

      // why here no need to add class_name::variable_name
      double ball_pos_x;  
      double ball_pos_y;
      double ball_velo_x;
      double ball_velo_y;
      bool measure;

      int unmeasured_frames_;

      int pid_fre_factor_;

    public:
      BallController();

      ~BallController();

      bool init(const Eigen::Vector4d &x, const BallControl::BallType ballType);

      bool init(const BallControl::BallType ballType);
      
      Eigen::Vector2d update(const double &time, const Eigen::Vector2d &u);

      Eigen::Vector4d getState();

      void setRunning() {controllerState_ = ControllerState::RUNNING;};
    
    private:
      void pubBallTF();

      void ball_pos_vel_Callback(const ball_controller::PosVel2D ball_pos_vel);

      bool initK();

      void pubState();

      void pubModelState();

      void pubPlateAngel(const Eigen::Vector2d &u);

      Eigen::Vector2d pid();


  };
}

#endif //PINGPONG_BALL_CONTROLLER_BALL_CONTROLLER