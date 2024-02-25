#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace state_action_factory
{
    Eigen::Vector2d encodeBallStateGrid(const Vector4d &x_ball, int size, int minCoord, int maxCoord)
    {
        // px vx py vy
        // set range from -0.2 ~ 0.2; but set all values out of as broader
        int discretized_ball_pos_x = discretizeCoordinate(x_ball(0), size, minCoord, maxCoord);
        int discretized_ball_pos_y = discretizeCoordinate(x_ball(2), size, minCoord, maxCoord);
        
        Eigen::Vector2d ball_pos;
        ball_pos<<discretized_ball_pos_x, discretized_ball_pos_y;
        return ball_pos;
    }

    Eigen::Vector2d encodeEndeffectorState(const Vector2d &EE_pos_r, int size)
    {
        // Axis 0: Rotation around the X-axis (roll),  Axis 1: Rotation around the Y-axis (pitch) 
        int discretized_EE_euler_x = discretizeCoordinate(EE_pos_r(0), size, minCoord, maxCoord);
        int discretized_EE_euler_y = discretizeCoordinate(EE_pos_r(1), size, minCoord, maxCoord);
        
        Eigen::Vector2d EE_euler;
        EE_euler<<discretized_EE_euler_x, discretized_EE_euler_x;
        return EE_euler;
    }

    int discretizeCoordinate(double coordinate, int size, double minCoord, double maxCoord) {
        // Define the continuous range boundaries
        // const double minCoord = -0.25;
        // const double maxCoord = 0.25;
        const double range = maxCoord - minCoord; // Total continuous range

        // Check if the coordinate is out of the continuous range boundaries
        if (coordinate <= minCoord) {
            return 0; // Lower boundary of the discrete range
        } else if (coordinate >= maxCoord) {
            return size - 1; // Upper boundary of the discrete range
        }

        // Normalize the coordinate to a [0, 1] range
        double normalizedCoord = (coordinate - minCoord) / range;

        // Scale the normalized coordinate to the [0, size-1] range and convert to an integer
        int discreteValue = static_cast<int>(normalizedCoord * (size - 1));

        return discreteValue;
    }



}