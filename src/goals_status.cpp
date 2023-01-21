#include <ros/ros.h>
#include <assignment_rt1_2/Goals.h>

// Declare node handle:
ros::NodeHandle *nh;

bool get_goals(assignment_rt1_2::Goals::Request &req, assignment_rt1_2::Goals::Response &res) {
    int gr, gc;
    if (!(*nh).getParam("/goals_reached", gr)) {
        ROS_INFO("Could not retrieve param /goals_succeeded.");
        return false;   
    }    
    if (!(*nh).getParam("/goals_cancelled", gc)) {
        ROS_INFO("Could not retrieve param /goals_cancelled.");
        return false; 
    }

    std::cout << "Goals reached: " << gr << std::endl;
    std::cout << "Goals cancelled: " << gc << std::endl;
    std::cout << "------------------------" << std::endl;
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "goals_server");

    // Initialize node handle:
    nh = new ros::NodeHandle;

    // Service server:
    ros::ServiceServer ss = (*nh).advertiseService("/goals", get_goals);

    ROS_INFO("Service server initialized.");

    ros::spin();

    return 0;
}