#include <ros/ros.h>
#include <assignment_rt1_2/RobotStatus.h>

float x, y, vel_x, vel_y, goal_x, goal_y;
int c = 0;
float acc_vx = 0, acc_vy = 0;


void get_status(const assignment_rt1_2::RobotStatus::ConstPtr &msg) {
    x = msg->x;
    y = msg->y;
    if (c > 0) {
        acc_vx += vel_x;
        acc_vy += vel_y;
        c++;
    }
    vel_x = msg->vel_x;
    vel_y = msg->vel_y;
    goal_x = msg->goal_x;
    goal_y = msg->goal_y;
    print_msg();
}

void print_msg() {
    std::cout << "ROBOT STATUS:" << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "y = " << y << std::endl;
    std::cout << "vel_x = " << vel_x << std::endl;
    std::cout << "vel_y = " << vel_y << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "ADVANCED STATUS:" << std::endl;
    std::cout << "Distance to goal: " << sqrt((goal_x - x)*(goal_x - x) + (goal_y - y)*(goal_y - y)) << std::endl;
    float av_x = (acc_vx + vel_x) / (c + 1);
    float av_y = (acc_vy + vel_y) / (c + 1);
    std::cout << "Average velocity in x: " << av_x << std::endl;
    std::cout << "Average velocity in y: " << av_y << std::endl;
    std::cout << "Average speed: " << sqrt(av_x*av_x + av_y*av_y) << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "======================================================" << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_status");

    // Node handler:
    ros::NodeHandle nh;

    // Check arguments for rate and goal position:
    ros::Rate *r;

    if (argc == 2) {
        r = new ros::Rate((double) atof(argv[1]));
    } else {
        ROS_INFO("Wrong number of arguments: (Rate).");
        exit(1);
    }

    // Subscribe:
    ros::Subscriber sub = nh.subscribe("robot/status", 1, get_status);

    while(ros::ok()) {
        
        // Check for callbacks:
        ros::spinOnce();

        // Sleep:
        r->sleep();
    }

    return 0;
}