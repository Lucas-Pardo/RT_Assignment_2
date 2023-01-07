#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2022/PlanningAction.h>

int main (int argc, char **argv) {

    ros::init(argc, argv, "test_bug0");

    // Create action client
    actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);

    // Wait for server
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

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
        ROS_INFO("Wrong number of arguments: (Rate) (goal_x goal_y). Starting with default values");
        goal_x = 2.0;
        goal_y = -5.0;
    }

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

    while (!finished && !ros::isShuttingDown()) {

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
                ac.cancelGoal();
                ROS_INFO("Goal cancelled.");
                break;
            }
        }

        // Wait for finish status
        finished = ac.waitForResult(r->expectedCycleTime());
    }

    if (finished) ROS_INFO("Goal reached.");

    return 0;

}