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

    ros::Publisher true_position_pb = nh.advertise<geometry_msgs::Vector3Stamped>("true_ball_position", 1);
    ros::Publisher true_velocity_pb = nh.advertise<geometry_msgs::Vector3Stamped>("true_ball_velocity", 1);

    ros::Publisher meas_position_pb = nh.advertise<geometry_msgs::Vector3Stamped>("meas_ball_position", 1);

    geometry_msgs::Vector3Stamped ball_position;
    geometry_msgs::Vector3Stamped ball_velocity;
    geometry_msgs::Vector3Stamped ball_acceleration;

    geometry_msgs::Vector3Stamped true_ball_position;
    geometry_msgs::Vector3Stamped true_ball_velocity;

    geometry_msgs::Vector3Stamped meas_ball_position;
    geometry_msgs::Vector3Stamped meas_ball_velocity;

    auto kalman_filter  = BallControl::KalmanFilter();

    kalman_filter.gerParam();

    double t = 0;
    double t_prev = 0;

    Eigen::Vector2d u(0, 0);

    while (ros::ok())
    {
        // Eigen::Vector2d zx(0.5 + 0.5 * 9.81 * t * t, 0.1 + 0.5 * 2.0 * t * t); 
        Eigen::Vector2d zx(0.5 + 0.5 * 9.81 * sin(t), 0.1 + 0.5 * 2.0 * cos(t));

        true_ball_position.header.stamp = ros::Time::now();
        true_ball_velocity.header.stamp = ros::Time::now();

        true_ball_position.vector.x = zx(0);
        true_ball_position.vector.y = zx(1);

        true_ball_velocity.vector.x = 0.5 * 9.81 * cos(t);
        true_ball_velocity.vector.y = - 0.5 * 2.0 * sin(t);

        true_position_pb.publish(true_ball_position);
        true_velocity_pb.publish(true_ball_velocity);

        zx(0) += 0.1 * ((double) rand() / (RAND_MAX) - 0.5);
        zx(1) += 0.1 * ((double) rand() / (RAND_MAX) - 0.5);
        ROS_INFO_STREAM("zx: \n" << zx);

        meas_ball_position.header.stamp = ros::Time::now();

        if (!kalman_filter.isInitialized())
        {
            Eigen::Matrix<double,8,1> x0 = Eigen::Matrix<double,8,1>::Zero();
            x0(0) = zx(0);
            x0(4) = zx(1);
            kalman_filter.init(x0);
        } else
        {
            kalman_filter.predict();
        }

        if (t - t_prev >= 0.03)
        {
            kalman_filter.updatePos(zx);    
            ROS_INFO_STREAM("*****************************************************");
            t_prev = t;

            meas_ball_position.vector.x = zx(0);
            meas_ball_position.vector.y = zx(1);
        }
        
        meas_position_pb.publish(meas_ball_position);

        Eigen::Vector2d za(-0.5 * 9.81 * sin(t), - 0.5 * 2.0 * cos(t)); 
        za(0) += 0.01 * ((double) rand() / (RAND_MAX) - 0.5);
        za(1) += 0.01 * ((double) rand() / (RAND_MAX) - 0.5);

        kalman_filter.updateAcc(za);

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