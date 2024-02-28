#include <ball_controller/ball_traj_planner.h>

namespace BallControl
{
    TrajPlanner::TrajPlanner():
    x_d_(Eigen::Vector4d::Zero()),
    circle_(false)
    {
        position_desired_sub_ = nh_.subscribe("ball_position_desired", 1000, &TrajPlanner::x_d_Callback, this);
        circle_sub_ = nh_.subscribe("ball_circle", 1000, &TrajPlanner::circle_Callback, this);
    }

    TrajPlanner::~TrajPlanner()
    {

    }

    Eigen::Vector4d TrajPlanner::update(const double &time)
    {   
        if (circle_)
        {
            circle(time);
        }
        return x_d_;
    }

    void TrajPlanner::circle(const double &t)
    {
        double w = M_PI_4;
        x_d_(0) = 0.1 * cos(w * t);
        x_d_(1) = -0.1 * w * sin(w * t); 
        x_d_(2) = 0.1 * sin(w * t);
        x_d_(3) = 0.1 * w * cos(w * t);
    }

    void TrajPlanner::x_d_Callback(const geometry_msgs::Point x_d)
    {
        x_d_(0) = x_d.x;
        x_d_(2) = x_d.y;
    }

    void TrajPlanner::circle_Callback(const std_msgs::Bool circle)
    {
        circle_ = circle.data;
    }
}
















