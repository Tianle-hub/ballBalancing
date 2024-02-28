#ifndef PINGPONG_BALL_TRAJECTORY_PLANNAR_BALL_TRAJECTORY_PLANNAR
#define PINGPONG_BALL_TRAJECTORY_PLANNAR_BALL_TRAJECTORY_PLANNAR

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace BallControl
{
    class TrajPlanner
    {
        private:
        Eigen::Vector4d x_d_;

        ros::NodeHandle nh_;

        ros::Subscriber position_desired_sub_;

        public:
        TrajPlanner();

        ~TrajPlanner();

        Eigen::Vector4d update(const double &time);

        private:

        void x_d_Callback(const geometry_msgs::Point x_d);
         
        void circle(const double &t);
    };
}

#endif //PINGPONG_BALL_TRAJECTORY_PLANNAR_BALL_TRAJECTORY_PLANNAR