#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_rt1_2/RobotStatus.h>
#include <nav_msgs/Odometry.h>
#include <assignment_rt1_2/Goals.h>

/**
 * \file move_to_goal.cpp
 * \brief Action client to move the robot to a goal
 * \author Lucas Pardo Bernardi
 * \version 1.1
 * \date 10/03/2023
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *  º /odom
 * 
 * Publishes to: <BR>
 * º /robot/status
 * 
 * Action client: <BR>
 * º /reaching_goal
 * 
 * Description :
 * 
 * This node implements an action client to control the execution of the action server.
 * It takes as arguments: (Rate) goal_x goal_y
 * where Rate is the rate (Hz) at which the node reads and publishes messages,
 * goal_x is the X position of the goal and goal_y the Y position. The rate can be skipped
 * to use the default value of 5 Hz.
 * 
 * The node reads messages to obtain the status of the robot and publishes a new message
 * containing the information read and the goal position. While the node is running
 * it also detects inputs to allow the user to cancel the action. After the action is
 * finished, whether succeded or cancelled, the node calls the service /goals to print 
 * the current number of goals reached and cancelled.
**/

float x; ///< Local copy of the x position of the robot
float y; ///< Local copy of the y position of the robot
float vel_x; ///< Local copy of the x component of the velocity
float vel_y; ///< Local copy of the y component of the velocity

/**
 * \brief Callback function to read robot status
 * \param msg The pointer to the message sent in the topic
 * 
 * This function is used to read the messages published in the topic "/odom"
 * and store the desired information in the global variables
**/
void get_status(const nav_msgs::Odometry::ConstPtr &msg) {
    x = (float) msg->pose.pose.position.x;
    y = (float) msg->pose.pose.position.y;
    vel_x = (float) msg->twist.twist.linear.x;
    vel_y = (float) msg->twist.twist.linear.y;
}

int main (int argc, char **argv) {
    
    ros::init(argc, argv, "test_bug0");

    // Node handler:
    ros::NodeHandle nh;

    // Create action client
    actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);

    // Wait for server
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    // Publisher:
    ros::Publisher pub;
    pub = nh.advertise<assignment_rt1_2::RobotStatus>("/robot/status", 1);
    assignment_rt1_2::RobotStatus rs;

    // Subscriber:
    ros::Subscriber sub = nh.subscribe("/odom", 1, get_status);

    ROS_INFO("Action server started, sending goal.");
    assignment_2_2022::PlanningGoal goal;
    double goal_x, goal_y;
    ros::Rate *r;

    // Check number of arguments
    if (argc == 4) {
        r = new ros::Rate(atof(argv[1]));
        goal_x = atof(argv[2]);
        goal_y = atof(argv[3]);
    } else if (argc == 3) {
        r = new ros::Rate(5);
        goal_x = atof(argv[1]);
        goal_y = atof(argv[2]);
    } else {
        ROS_INFO("Wrong number of arguments: (Rate) goal_x goal_y. Starting with default values (5) (2, -5).");
        r = new ros::Rate(5);
        goal_x = 2.0;
        goal_y = -5.0;
    }

    cycle:

    // Send goal
    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    ac.sendGoal(goal);

    ROS_INFO("Goal sent, press q to cancel task.");

    bool finished = ac.waitForResult(r->expectedCycleTime());

    // Variables for select()
    fd_set rfds;
    int retval;
    std::string cmd;
    struct timeval tv = {0, 0};

    while (!finished && ros::ok()) {

        // Create the set of read fds:
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);

        retval = select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv);
        if (retval < 0 && errno != EINTR) {
            std::cerr << "Error in select: " << strerror(errno) << "\n";
            break;
        }
        else if (retval) {
            std::cin >> cmd;
            if (cmd == "q") {
                // Cancel goal
                ac.cancelGoal();
                ROS_INFO("Goal cancelled.");
                // Change param
                int gc;
                nh.getParam("/goals_cancelled", gc);
                nh.setParam("/goals_cancelled", gc + 1);
                break;
            }
        }

        // Check for callbacks:
        ros::spinOnce();

        // Publish status:
        rs.x = x;
        rs.y = y;
        rs.vel_x = vel_x;
        rs.vel_y = vel_y;
        rs.goal_x = goal_x;
        rs.goal_y = goal_y;
        pub.publish(rs);

        // Wait for finish status
        finished = ac.waitForResult(r->expectedCycleTime());
    }

    if (finished) {
        ROS_INFO("Goal reached.");
        // Change param
        int gr;
        nh.getParam("/goals_reached", gr);
        nh.setParam("/goals_reached", gr + 1);
    } 

    // Call goals service
    ros::ServiceClient client = nh.serviceClient<assignment_rt1_2::Goals>("/goals");
    assignment_rt1_2::Goals g;
    client.call(g);

    // Exit routine with retry option:
    while (true) {
        std::cout << "Program finished. Write 'r' to retry with another goal or 'e' to exit:" << std::endl;
        std::string buf;
        std::cin >> buf;
        if (buf == "e" || buf == "E") break;
        if (buf == "r" || buf == "R") {
            std::cout << "Enter new goal_x:" << std::endl;
            std::cin >> buf;
            goal_x = atof(buf.c_str());
            std::cout << "Enter new goal_y:" << std::endl;
            std::cin >> buf;
            goal_y = atof(buf.c_str());
            goto cycle;
        }
    }

    return 0;

}