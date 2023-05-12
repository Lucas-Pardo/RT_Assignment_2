#include <ros/ros.h>
#include <assignment_rt1_2/Goals.h>

/**
 * \file goals_status.cpp
 * \brief Service server to print number of goals
 * \author Lucas Pardo Bernardi
 * \version 1.1
 * \date 13/03/2023
 * 
 * \details
 * 
 * \param [in] goals_reached Number of goals reached
 * \param [in] goals_cancelled Number of goals cancelled
 * 
 * Service server: <BR>
 * º /goals
 * 
 * Description :
 * 
 * This node implements a service server that when called prints the
 * number of goals reached and the number of goals cancelled to the terminal.
**/

// Declare node handle:
ros::NodeHandle *nh; ///< Global declaration of node handler


/**
 * \brief Callback function that gets and prints goals
 * \param req Pointer to the request of the service
 * \param res Pointer to the response of the service
 * 
 * This function uses the information contained in the parameter server
 * to print the number of goals reached and cancelled. This function is
 * executed when the service "/goals" is called.
**/
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