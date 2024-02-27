#ifndef STATE_ACTION_H
#define STATE_ACTION_H

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>



namespace q_learning
{
    int inner_counter = 0;
    int learning_freq = 20;
    double plate_x_delta_onestep = 0;
    double plate_y_delta_onestep = 0;
    // Variables for Q learning
    Eigen::Vector2d plate_angle = Eigen::Vector2d::Zero();
    // Eigen::Vector2d last_plate_angle = Eigen::Vector2d::Zero();


    double ball_range = 0.2;
    double robot_rotation_range = 0.06;  // rotation maximum angle in radian
    int num_ball_grid = 20;
    int num_ball_polar_theta = 8;
    int num_ball_polar_radius = 5;
    int num_robot = 8;
    double unit_robot_rotate = robot_rotation_range/double(num_robot);
    int num_state = num_robot*num_robot*num_ball_polar_theta*num_ball_polar_radius;
    int num_action = 9;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_state, num_action);
    Eigen::Vector2d delta_robot_rotate;
    bool new_action = false;


    bool Q_init = false;
    const int numEpisodes = 1000; // Total number of episodes to run
    const int maxSteps = 100; // Maximum steps per episode
    int episode = 0;
    int step = 0;
    int currentState;
    int nextState;
    int action;
    bool done;
    double reward = 0;
    double Reward;

    const double alpha = 0.1; // Learning rate
    const double gamma = 0.99; // Discount factor
    double epsilon = 0.3; // Epsilon for epsilon-greedy policy


    Eigen::Vector2d encodeBallStateGrid(Eigen::Vector4d &x_ball, int size, double minCoord, double maxCoord);
    Eigen::Vector4d encodeBallPosVeloPolar(Eigen::Vector4d &x_ball, int angle_size, int dis_size, double minCoord, double maxCoord);
    Eigen::Vector2d encodeEndeffectorState(Eigen::Vector2d &EE_pos_r, int size);
    int discretizeCoordinate(double coordinate, int size, double minCoord, double maxCoord);
    void action2plateAngleAbsolute(int action);

    void randomEpisodePlate()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-robot_rotation_range, robot_rotation_range);
        plate_angle << dis(gen), dis(gen);
        ROS_INFO_STREAM("Init plate_angle \n"<<plate_angle);
    }


    int getState(Eigen::Vector4d &discretized_ball_pos_velo_polar,
                Eigen::Vector2d &discretized_EE_euler,
                int num_ball_polar_theta, int num_ball_polar_radius, int num_robot)
    {
        int state_index = discretized_ball_pos_velo_polar(0)*num_ball_polar_theta*num_ball_polar_radius*num_robot 
                      + discretized_ball_pos_velo_polar(1)*num_ball_polar_radius*num_robot 
                      + discretized_EE_euler(0)*num_robot 
                      + discretized_EE_euler(1);

        return state_index;
    }

    double getReward(Eigen::Vector4d &discretized_ball_pos_velo_polar, Eigen::Vector2d &discretized_EE_euler, int ball_range)
    {
        double reward;
        int encodedBallPosAngle = discretized_ball_pos_velo_polar(0);
        int encodedBallPosDis = discretized_ball_pos_velo_polar(1);
        int encodedVeloAngle = discretized_ball_pos_velo_polar(2);
        double VeloRadius = discretized_ball_pos_velo_polar(3);
        int encodedEEx = discretized_EE_euler(0);
        int encodedEEy = discretized_EE_euler(1);

        if (encodedBallPosDis == 0 
            && (encodedEEx<=num_robot/2+1 || encodedEEx>=num_robot/2-1) 
            && (encodedEEy<=num_robot/2+1 || encodedEEy>=num_robot/2-1) ){
            reward = 100; 
            done = true;
        }
        else if (encodedBallPosDis == 0) reward = 1;
        else if (encodedBallPosDis == 1) reward = -1;
        else if (encodedBallPosDis == 2) reward = -2;
        else if (encodedBallPosDis == 3) reward = -4;  // check whether not exceed
        else reward = -10;

        return reward;
    }

    int chooseAction(int state, const Eigen::MatrixXd &Q, double epsilon) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        std::uniform_int_distribution<> actionDis(0, num_action - 1);

        if (dis(gen) > epsilon) {
            // Choose the best action for the current state
            Eigen::MatrixXd::Index maxIndex;
            Q.row(state).maxCoeff(&maxIndex);
            return static_cast<int>(maxIndex);
        } else {
            // Choose a random action
            return actionDis(gen);
        }
    }

    Eigen::Vector2d action2plateAngle(int action){
        Eigen::Vector2d delta;
        if (action == 0){
            // stay still
            delta<<0, 0;
        }
        else if (action == 1){
            // up
            // plate_angle(0) -= unit_robot_rotate;
            // plate_angle(1) += 0;
            delta<<-unit_robot_rotate,0;
        }
        else if (action == 2){
            // up right
            // plate_angle(0) -= unit_robot_rotate;
            // plate_angle(1) += unit_robot_rotate;
            delta<<-unit_robot_rotate,unit_robot_rotate;
        }
        else if (action == 3){
            // right
            // plate_angle(0)+= 0;
            // plate_angle(1)+= unit_robot_rotate;
            delta<<0,unit_robot_rotate;
        }
        else if (action == 4){
            // down right
            // plate_angle(0)+= unit_robot_rotate;
            // plate_angle(1)+= unit_robot_rotate;
            delta<<unit_robot_rotate,unit_robot_rotate;
        }
        else if (action == 5){
            // down 
            // plate_angle(0)+= unit_robot_rotate;
            // plate_angle(1)+= 0;
            delta<<unit_robot_rotate,0;
        }
        else if (action == 6){
            // down left
            // plate_angle(0)+= unit_robot_rotate;
            // plate_angle(1)-= unit_robot_rotate;
            delta<<unit_robot_rotate,-unit_robot_rotate;
        }
        else if (action == 7){
            // left 
            // plate_angle(0)+= 0;
            // plate_angle(1)-= unit_robot_rotate;
            delta<<0,-unit_robot_rotate;
        }
        else if (action == 8){
            // up left
            // plate_angle(0)-= unit_robot_rotate;
            // plate_angle(1)-= unit_robot_rotate;
            delta<<-unit_robot_rotate,-unit_robot_rotate;
        }
        if (plate_angle(0) >= robot_rotation_range) delta(0) = 0;
        else if (plate_angle(0) <= -robot_rotation_range) delta(0) = 0;
        else if (plate_angle(1) >= robot_rotation_range) delta(1) = 0;
        else if (plate_angle(1) <= -robot_rotation_range) delta(1) = 0;

        // if (plate_angle(0) >= robot_rotation_range) plate_angle(0) = robot_rotation_range;
        // else if (plate_angle(0) <= -robot_rotation_range) plate_angle(0) = -robot_rotation_range;
        // else if (plate_angle(1) >= robot_rotation_range) plate_angle(1) = robot_rotation_range;
        // else if (plate_angle(1) <= -robot_rotation_range) plate_angle(1) = -robot_rotation_range;

    return delta;
    }


    void action2plateAngleAbsolute(int action){
        if (action == 0){
            // stay still
            plate_angle(0) += 0;
            plate_angle(1) += 0;
        }
        else if (action == 1){
            // up
            plate_angle(0) -= unit_robot_rotate;
            plate_angle(1) += 0;
        }
        else if (action == 2){
            // up right
            plate_angle(0) -= unit_robot_rotate;
            plate_angle(1) += unit_robot_rotate;
        }
        else if (action == 3){
            // right
            plate_angle(0)+= 0;
            plate_angle(1)+= unit_robot_rotate;
        }
        else if (action == 4){
            // down right
            plate_angle(0)+= unit_robot_rotate;
            plate_angle(1)+= unit_robot_rotate;
        }
        else if (action == 5){
            // down 
            plate_angle(0)+= unit_robot_rotate;
            plate_angle(1)+= 0;
        }
        else if (action == 6){
            // down left
            plate_angle(0)+= unit_robot_rotate;
            plate_angle(1)-= unit_robot_rotate;
        }
        else if (action == 7){
            // left 
            plate_angle(0)+= 0;
            plate_angle(1)-= unit_robot_rotate;
        }
        else if (action == 8){
            // up left
            plate_angle(0)-= unit_robot_rotate;
            plate_angle(1)-= unit_robot_rotate;
        }
        // if (plate_angle(0) >= robot_rotation_range) delta(0) = 0;
        // else if (plate_angle(0) <= -robot_rotation_range) delta(0) = 0;
        // else if (plate_angle(1) >= robot_rotation_range) delta(1) = 0;
        // else if (plate_angle(1) <= -robot_rotation_range) delta(1) = 0;

        if (plate_angle(0) >= robot_rotation_range) plate_angle(0) = robot_rotation_range;
        else if (plate_angle(0) <= -robot_rotation_range) plate_angle(0) = -robot_rotation_range;
        else if (plate_angle(1) >= robot_rotation_range) plate_angle(1) = robot_rotation_range;
        else if (plate_angle(1) <= -robot_rotation_range) plate_angle(1) = -robot_rotation_range;

    }

    void updateQ(Eigen::MatrixXd &Q, int state, int action, double reward, int nextState) {
        double maxNextQ = Q.row(nextState).maxCoeff(); // Max Q-value for the next state
        Q(state, action) += alpha * (reward + gamma * maxNextQ - Q(state, action));
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
        double radius = sqrt(std::pow(x_ball(1),2) + std::pow(x_ball(3),2));
        double theta = atan2(x_ball(3), x_ball(1));
        polar_vel<<radius, theta;
        normalizedTheta = polar_vel(1) < 0 ? (2 * M_PI + polar_vel(1)) : polar_vel(1);

        int encodedVeloAngle = static_cast<int>((normalizedTheta / (2 * M_PI)) * angle_size); // share same angle size wiz pos

        Eigen::Vector4d encodedPosVelo;
        encodedPosVelo<<encodedPosAngle, encodedPosDis, encodedVeloAngle, radius;
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