#include <ros/ros.h>
#include <assignment_rt1_2/Goals.h>

// Declare node handle:
ros::NodeHandle *nh;

bool get_goals(assignment_rt1_2::Goals::Request &req, assignment_rt1_2::Goals::Response &res) {
    std::string buf;
    int suc, can;
    if (!(*nh).getParam("goals_succeeded", buf)) {
        ROS_INFO("Could not retrieve param /goals_succeeded.");
        return false;   
    }
    suc = atoi(buf.c_str());
    
    if (!(*nh).getParam("goals_cancelled", buf)) {
        ROS_INFO("Could not retrieve param /goals_cancelled.");
        return false; 
    }
    can = atoi(buf.c_str());

    std::cout << "Goals reached: " << suc << std::endl;
    std::cout << "Goals cancelled: " << can << std::endl;
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