#include <ball_controller/ball_model.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{

    ros::init(argc,argv,"testBallModel",ros::init_options::AnonymousName);
    ros::Time::init();

    BallController::BallModel ball;
    if(!ball.initModel(Eigen::Vector4d(0, 0, 0, 0), Eigen::Vector2d(0, 0)))
    {
        return -1;
    }

    double t = 0;

    while (true)
    {
      Eigen::Vector4d x = ball.update(t, Eigen::Vector2d(0.01, 0.01));

      ROS_INFO_STREAM("t : " << t);
      ROS_INFO_STREAM("x : " << x(0));
      ROS_INFO_STREAM("v : " << x(1));
      ROS_INFO_STREAM("y : " << x(2));
      ROS_INFO_STREAM("v : " << x(3));
      ros::Duration(0.002).sleep();
      t += 0.002;
    }

    return 0;
}