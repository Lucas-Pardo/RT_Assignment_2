#include <ros/ros.h>
#include <assignment_rt1_2/RobotStatus.h>

/**
 * \file robot_status.cpp
 * \brief Node that prints robot status information
 * \author Lucas Pardo Bernardi
 * \version 1.1
 * \date 13/03/2023
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *  º /robot/status
 * 
 * Description :
 * 
 * This node reads the information contained in the custom message
 * published in the topic "/robot/status" and prints it on the terminal
 * at a certain rate. It takes as argument the rate at which the node works (Hz).
**/
float x; ///< Local copy of the x position of the robot
float y; ///< Local copy of the y position of the robot
float vel_x; ///< Local copy of the x component of the velocity
float vel_y; ///< Local copy of the y component of the velocity
float goal_x = 0; ///< Local copy of the x position of the current goal
float goal_y = 0; ///< Local copy of the y position of the current goal
int c = 0; ///< Number of messages received for current goal
float acc_vx = 0; ///< Accumulation of the x component of the velocity
float acc_vy = 0; ///< Accumulation of the y component of the velocity

/**
 * \brief Function that computes and prints information to the terminal
 * 
 * This function uses the information contained in the global variables
 * to print the robot position, velocity, distance to the goal, average
 * speed in each direction and total average speed.
**/
void print_msg() {
    std::cout << "ROBOT STATUS:" << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "y = " << y << std::endl;
    std::cout << "vel_x = " << vel_x << std::endl;
    std::cout << "vel_y = " << vel_y << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "ADVANCED STATUS:" << std::endl;
    std::cout << "Distance to goal: " << std::sqrt((goal_x - x)*(goal_x - x) + (goal_y - y)*(goal_y - y)) << std::endl;
    float av_x = acc_vx / c;
    float av_y = acc_vy / c; 
    std::cout << "Average speed in x: " << av_x << std::endl;
    std::cout << "Average speed in y: " << av_y << std::endl;
    std::cout << "Average speed: " << std::sqrt(av_x*av_x + av_y*av_y) << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "======================================================" << std::endl;
}

/**
 * \brief Callback function to print robot status
 * \param msg The pointer to the message sent in the topic
 * 
 * This function is used to update the global variables with the
 * new information available in the message and at the end it calls
 * the previous function "print_msg" to print this new information
 * to the terminal.
**/
void get_status(const assignment_rt1_2::RobotStatus::ConstPtr &msg) {
    // Check if goal changed:
    float eps = 1e-3;  // Threshold for the goal change
    if (std::abs(msg->goal_x - goal_x) > eps || std::abs(msg->goal_y - goal_y) > eps) {
        ROS_INFO("Goal change detected, resetting stats.");
        goal_x = msg->goal_x;
        goal_y = msg->goal_y;
        acc_vx = 0;
        acc_vy = 0;
    }
    // Copy parameters from the message:
    x = msg->x;
    y = msg->y;
    vel_x = msg->vel_x;
    vel_y = msg->vel_y;
    acc_vx += std::abs(vel_x);
    acc_vy += std::abs(vel_y);
    c++;
    print_msg();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_status");

    // Node handler:
    ros::NodeHandle nh;

    // Check arguments for rate:
    ros::Rate *r;

    if (argc == 2) {
        r = new ros::Rate((double) atof(argv[1]));
    } else {
        ROS_INFO("Wrong number of arguments: (Rate).");
        exit(1);
    }

    // Subscribe:
    ros::Subscriber sub = nh.subscribe("/robot/status", 1, get_status);

    while(ros::ok()) {
        
        // Check for callbacks:
        ros::spinOnce();

        // Sleep at the given rate:
        r->sleep();
    }

    return 0;
}