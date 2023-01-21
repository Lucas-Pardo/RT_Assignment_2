#include <ros/ros.h>
#include <assignment_rt1_2/RobotStatus.h>

float x, y, vel_x, vel_y, goal_x = 0, goal_y = 0;
int c = 0;
float acc_vx = 0, acc_vy = 0;

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