#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2022/PlanningAction.h>
#include <assignment_rt1_2/RobotStatus.h>
#include <nav_msgs/Odometry.h>
#include <assignment_rt1_2/Goals.h>

float x, y, vel_x, vel_y;

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

    // Send goal
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
        ROS_INFO("Wrong number of arguments: (Rate) (goal_x goal_y). Starting with default values (5) (2, -5).");
        r = new ros::Rate(5);
        goal_x = 2.0;
        goal_y = -5.0;
    }

    cycle:

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