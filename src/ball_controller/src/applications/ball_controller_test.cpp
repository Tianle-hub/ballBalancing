#include <ball_controller/ball_controller.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

int main(int argc, char **argv)
{

    ros::init(argc,argv,"testBallModel",ros::init_options::AnonymousName);
    ros::NodeHandle nh = ros::NodeHandle();
    ros::Time::init();

    BallControl::BallController ball;
    if(!ball.initModel(Eigen::Vector4d(0.2, 0.1, 0.1, 0.1), Eigen::Vector2d(0, 0)))
    {
        return -1;
    }

    ros::Publisher position_pb = nh.advertise<geometry_msgs::Vector3Stamped>("ball_position", 1);
    ros::Publisher velocity_pb = nh.advertise<geometry_msgs::Vector3Stamped>("ball_velocity", 1);

    geometry_msgs::Vector3Stamped ball_position;
    geometry_msgs::Vector3Stamped ball_velocity;

    double t = 0;

    Eigen::Vector2d u(0, 0);

    while (ros::ok())
    {
      Eigen::Vector4d x = ball.updateModel(t, u);

      u = ball.update(x);

      ball_position.header.stamp = ros::Time::now();
      ball_velocity.header.stamp = ros::Time::now();

      ball_position.vector.x = x(0);
      ball_position.vector.y = x(2);

      ball_velocity.vector.x = x(1);
      ball_velocity.vector.y = x(3);

      position_pb.publish(ball_position);
      velocity_pb.publish(ball_velocity);

      // ROS_INFO_STREAM("t : " << t);
      // ROS_INFO_STREAM("x : " << x(0));
      // ROS_INFO_STREAM("v : " << x(1));
      // ROS_INFO_STREAM("y : " << x(2));
      // ROS_INFO_STREAM("v : " << x(3));
      t += 0.002;
      ros::Duration(0.002).sleep();
      ros::spinOnce();
    }

    return 0;
}