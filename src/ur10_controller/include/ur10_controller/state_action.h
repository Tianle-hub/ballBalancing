#ifndef STATE_ACTION_H
#define STATE_ACTION_H

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
namespace state_action_factory
{
    Eigen::Vector2d encodeBallStateGrid(Eigen::Vector4d &x_ball, int size, double minCoord, double maxCoord);
    Eigen::Vector4d encodeBallPosVeloPolar(Eigen::Vector4d &x_ball, int angle_size, int dis_size, double minCoord, double maxCoord);
    Eigen::Vector2d encodeEndeffectorState(Eigen::Vector2d &EE_pos_r, int size);
    int discretizeCoordinate(double coordinate, int size, double minCoord, double maxCoord);

    double getReward(Eigen::Vector4d discretized_ball_pos_velo_polar, Eigen::Vector2d discretized_EE_euler)
    {
        double reward;
        int encodedBallPosAngle = discretized_ball_pos_velo_polar(0);
        int encodedBallPosDis = discretized_ball_pos_velo_polar(1);
        int encodedEEx = discretized_EE_euler(0);
        int encodedEEy = discretized_EE_euler(1);

        return reward;
    }

    Eigen::Vector2d encodeBallStateGrid(Eigen::Vector4d &x_ball, int size, double minCoord, double maxCoord)
    {
        // px vx py vy
        // set range from -0.2 ~ 0.2; but set all values out of as broader
        int discretized_ball_pos_x = discretizeCoordinate(x_ball(0), size, minCoord, maxCoord);
        int discretized_ball_pos_y = discretizeCoordinate(x_ball(2), size, minCoord, maxCoord);
        
        Eigen::Vector2d ball_pos;
        ball_pos<<discretized_ball_pos_x, discretized_ball_pos_y;
        return ball_pos;
    };

    Eigen::Vector4d encodeBallPosVeloPolar(Eigen::Vector4d &x_ball, int angle_size, int dis_size, double minCoord, double maxCoord)
    {
        // Position part
        Eigen::Vector2d polar_pos;
        polar_pos<<sqrt(std::pow(x_ball(0), 2) + std::pow(x_ball(2),2)), atan2(x_ball(2), x_ball(0));
        double normalizedTheta = polar_pos(1) < 0 ? (2 * M_PI + polar_pos(1)) : polar_pos(1);

        // Scale the normalized angle to the range 0 to n
        // Get discretized angle coordinate
        int encodedPosAngle = static_cast<int>((normalizedTheta / (2 * M_PI)) * angle_size);
        // Get discretized distance coordinate
        int encodedPosDis = discretizeCoordinate(polar_pos(0), dis_size, minCoord, maxCoord); // set init as size:5, min:0, max:0.2

        // Velocity part
        Eigen::Vector2d polar_vel;
        polar_vel<<sqrt(std::pow(x_ball(1),2) + std::pow(x_ball(3),2)), atan2(x_ball(3), x_ball(1));
        normalizedTheta = polar_vel(1) < 0 ? (2 * M_PI + polar_vel(1)) : polar_vel(1);

        int encodedVeloAngle = static_cast<int>((normalizedTheta / (2 * M_PI)) * angle_size); // share same angle size wiz pos

        Eigen::Vector4d encodedPosVelo;
        encodedPosVelo<<encodedPosAngle, encodedPosDis, encodedVeloAngle, 0;
        return encodedPosVelo;
    }

    Eigen::Vector2d encodeEndeffectorState(Eigen::Vector2d &EE_pos_r, int size, double minCoord, double maxCoord)
    {
        // Axis 0: Rotation around the X-axis (roll),  Axis 1: Rotation around the Y-axis (pitch) 
        int discretized_EE_euler_x = discretizeCoordinate(EE_pos_r(0), size, minCoord, maxCoord);
        int discretized_EE_euler_y = discretizeCoordinate(EE_pos_r(1), size, minCoord, maxCoord);
        
        Eigen::Vector2d EE_euler;
        EE_euler<<discretized_EE_euler_x, discretized_EE_euler_y;
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

    Eigen::Vector2d readCommand(Eigen::Vector2d &plate_angle){
        char command;
        while (std::cin >> command) {
        switch (command) {
            case 'W':
            case 'w':
                std::cout << "Move Up" << std::endl;
                plate_angle(0) += 0.02;
                return plate_angle;
                break;
            case 'A':
            case 'a':
                std::cout << "Move Left" << std::endl;
                plate_angle(1) += 0.02;
                return plate_angle;
                break;
            case 'S':
            case 's':
                std::cout << "Move Down" << std::endl;
                plate_angle(0) -= 0.02;
                return plate_angle;
                break;
            case 'D':
            case 'd':
                std::cout << "Move Right" << std::endl;
                plate_angle(1) -= 0.02;
                return plate_angle;
                break;
            case 'Q':
            case 'q':
                std::cout << "Quitting..." << std::endl;
                return plate_angle; // Exit the loop and program
            default:
                std::cout << "Invalid input. Please use W, A, S, D to move, Q to quit." << std::endl;
        }
        std::cout << "Next command: ";
        }
        return plate_angle;
        
    }


};

#endif