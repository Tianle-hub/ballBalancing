#include <ball_controller/kalman_filter.h>
#include <random>
#include <geometry_msgs/Vector3Stamped.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"testKalmanFilter",ros::init_options::AnonymousName);
    ros::NodeHandle nh = ros::NodeHandle();
    ros::Time::init();

    ros::Publisher position_pb = nh.advertise<geometry_msgs::Vector3Stamped>("ball_position", 1);
    ros::Publisher velocity_pb = nh.advertise<geometry_msgs::Vector3Stamped>("ball_velocity", 1);
    ros::Publisher acceleration_pb = nh.advertise<geometry_msgs::Vector3Stamped>("ball_acceleration", 1);

    geometry_msgs::Vector3Stamped ball_position;
    geometry_msgs::Vector3Stamped ball_velocity;
    geometry_msgs::Vector3Stamped ball_acceleration;

    auto kalman_filter  = BallControl::KalmanFilter();

    kalman_filter.init(Eigen::Matrix<double,8,1>::Zero());

    double t = 0;
    double t_prev = 0;

    Eigen::Vector2d u(0, 0);

    while (ros::ok())
    {
        kalman_filter.predict();
        auto x = kalman_filter.getState();

        ROS_INFO_STREAM("t: \n" << t);
        ROS_INFO_STREAM("x: \n" << x.transpose());

        ball_position.header.stamp = ros::Time::now();
        ball_velocity.header.stamp = ros::Time::now();
        ball_acceleration.header.stamp = ros::Time::now();

        ball_position.vector.x = x(0);
        ball_position.vector.y = x(4);

        ball_velocity.vector.x = x(1);
        ball_velocity.vector.y = x(5);

        ball_acceleration.vector.x = x(2);
        ball_acceleration.vector.y = x(6);

        position_pb.publish(ball_position);
        velocity_pb.publish(ball_velocity);
        acceleration_pb.publish(ball_acceleration);

        t += 0.002;
        ros::Duration(0.002).sleep();
        ros::spinOnce();
    }
}