# Assignment 2 (RT1): Lucas Pardo (s5646719) 

## Prerequisites

This assignment is built upon the following [ros package](https://github.com/CarmineD8/assignment_2_2022). This base package implements the [bug 0 algorithm](https://en.wikipedia.org/wiki/Bug_algorithm) as an action server in python and makes use of Rviz and Gazebo to simulate everything. These two pieces of software are included in the full instalation of ROS Noetic. Python 3 must be the default version.

## Assignment description

The goal of the assignment is to create a standalone ros package based on the [base package](https://github.com/CarmineD8/assignment_2_2022) with the following characteristics:

* A node (A) implementing an action client that allows the user to set the goal of the robot (its position) or cancel it. This node also needs to publish the position and velocity of the robot as a custom message.

* A node (B) implementing a service server that when called prints the number of goals reached and cancelled.

* A node (C) that subscribes to the robot's position and velocity using the custom message published by node A and prints several stats such as distance to goal and average speed. The frequency at which it prints information can be set by the user.

* A launch file that starts the whole simulation and sets the different parameters needed.

## How to run the program

The whole simulation including the contents of this assignment can be run using the following command after initializing ros (*roscore*) and compiling the package (*catkin_make*):

```shell
roslaunch assignment_rt1_2 assignment.launch rate:=(R) goal_x:=(GX) goal_y:=(GY)
```

The values `(R)`, `(GX)` and `(GY)` are the user input. The launch file is explained later but the `rate` argument has a default value so `rate:=(R)` can be omitted.

## In-depth explanation of the code

Every cpp file is properly commented to understand what is being done but here we provide a more detailed explanation and the motive behind every decision.

### Node A (move_to_goal.cpp)

Setting the goal is easy and can be done just by passing the goal position as argument to the function (*argv*). This can be parsed and sent to the action server. However, we also must be able to cancel the goal at any time, which means that we need to check for user input periodically. We must also publish the custom message with the given rate.

#### **Pseudocode**

The basic pseudocode for node A can be the following:

```
goal, rate = parse arguments
send goal to action server

while action is not finished:
    if there is user input to cancel goal:
        cancel goal
        break loop
    check for callbacks ("/odom")
    publish custom message
    sleep at the given rate
```

Our code is a little different in the sense that it cycles itself after finishing to allow the user to input a new goal, so in reality, this pseudocode would be inside a loop.

#### **Argument parsing**

```cpp
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
```

As we can see in the code, it was designed to be able to run with different parameters. It can run with all three parameters given. The rate can be omitted, in which case the default rate of 5 Hz will be used. In any other case the arguments are considered to be incomplete, in which case a warning message is given and the node is executed with some default values.

This is not really useful for the complete execution of the assignment (using the launch file) since it will always have the three arguments, else the launch itself fails. Nevertheless, it is useful when the node is run manually.

#### **Checking user input**

An easy way to check for user input in non-blocking mode is using a `select` with zero timeout.

```cpp
// Variables for select()
fd_set rfds;
int retval;
std::string cmd;
struct timeval tv = {0, 0};

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
```

As we can see, we only read from the input (`cin`) if there is something available to read, and in this case, if that is the letter 'q' we cancel the goal (which is explained later).

#### **Main loop**

The main loop is where we continuously check for user input as explained, check for messages from `"/odom"`, and publish the custom message. The loop runs until the robot reaches the goal, ros is shutdown or the goal is cancelled (`break` in the previous code).

```cpp
bool finished = ac.waitForResult(r->expectedCycleTime());

while (!finished && ros::ok()) { 
    ...
    finished = ac.waitForResult(r->expectedCycleTime());
}
```

We take advantage of the function `waitForResult`, which can take as argument a duration to wait for, to make the program sleep for the given rate.

#### **Publisher and subscriber**

To obtain the information about the robot such as position and velocity we use the information published in the topic `"/odom"` through the callback function `get_status`:

```cpp
float x, y, vel_x, vel_y;

void get_status(const nav_msgs::Odometry::ConstPtr &msg) {
    x = (float) msg->pose.pose.position.x;
    y = (float) msg->pose.pose.position.y;
    vel_x = (float) msg->twist.twist.linear.x;
    vel_y = (float) msg->twist.twist.linear.y;
}
```

To publish the information we need to node C, we use the custom message defined in the *msg* folder, `RobotStatus.msg`:

```
float32 x
float32 y
float32 vel_x
float32 vel_y
float32 goal_x
float32 goal_y
```

* Notice that we also send the goal position as part of the message. The reason why is explained in [node C](#node-c-robot_statuscpp).

In node A we simply publish this information to the topic `"/robot/status"`:

```cpp
// Publisher:
ros::Publisher pub;
pub = nh.advertise<assignment_rt1_2::RobotStatus>("/robot/status", 1);
assignment_rt1_2::RobotStatus rs;
```

```cpp
// Publish status:
rs.x = x;
rs.y = y;
rs.vel_x = vel_x;
rs.vel_y = vel_y;
rs.goal_x = goal_x;
rs.goal_y = goal_y;
pub.publish(rs);
```

#### **Information about the goals**

As it is explained in the [assignment description](#assignment-description), node B must have access to the goals both reached and cancelled, information that is only available in the action server or client. That means that we need a way to send information from one node to the other. Since node B is a service server, the usual way to convey this information would be through a *request*, but for reasons explained later, we have decided to use the *parameter server*. We can use a *param* to hold the number of goals reached and another for the goals cancelled. So in both cases, we just read the current value of that *param* and set it to it plus one:

```cpp
// Change param
int gc;
nh.getParam("/goals_cancelled", gc);
nh.setParam("/goals_cancelled", gc + 1);
```


#### **Exit routine**

Exiting the main loop only occurs, under normal circunstances, after the robot has either reached the goal or it has been cancelled. If the `finished` flag is `true`, the goal has been reached and we change the *param*:

```cpp
if (finished) {
    ROS_INFO("Goal reached.");
    // Change param
    int gr;
    nh.getParam("/goals_reached", gr);
    nh.setParam("/goals_reached", gr + 1);
} 
```

After finishing, the program calls the service of node B to print the goals:

```cpp
// Call goals service
ros::ServiceClient client = nh.serviceClient<assignment_rt1_2::Goals>("/goals");
assignment_rt1_2::Goals g;
client.call(g);
```

As we said in the [pseudocode](#pseudocode), we designed the code to allow the user to input a new goal without exiting the program and having to call it again manually (rosrun ...). To do that, we create an infinite loop that is only broken when the user inputs "e" to exit or "r" to retry. After inputting "r" the user is then prompted to enter the new goal position:

```cpp
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
```

We make use of the `goto` function and the label `cycle` to jump to a previous point in the program, just before sending the goal:

```cpp
cycle:

goal.target_pose.pose.position.x = goal_x;
goal.target_pose.pose.position.y = goal_y;
ac.sendGoal(goal);
ROS_INFO("Goal sent, press q to cancel task.");
```

### Node B (goals_status.cpp)

As we explained in [node A](#node-a-move_to_goalcpp), we use *params* to save the amount of goals reached and cancelled to the *parameter server*. The node then creates the service server `"/goals"` and waits infinitely for calls:

```cpp
// Service server:
ros::ServiceServer ss = (*nh).advertiseService("/goals", get_goals);

ROS_INFO("Service server initialized.");

ros::spin();
```

The callback function `get_goals` just gets the *params* from the *parameter server* and prints them:

```cpp
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
```

Because the service just prints *params*, there is no need for a `request` or `response`. As such, the service `Goals.srv` defined in the *srv* folder is just an empty service:

```
---
```

#### **Other possible approaches**

The whole reason behind using the *parameter server* to hold the goals reached and cancelled is because we only have access to that information while the action client (node A) is running. We could suppose that node A will be run continuously, i.e. that the user will always use the retry functionality of the program, in which case we could have local variables counting the number of goals reached and cancelled and send them as part of the request like this:

```
int goals_reached
int goals_cancelled
---
```

That approach however, would not allow for manual calls of the service since the user would need to know the amount before. A better approach would be to have the local counting variables in node B and node A just sends a `bool` variable of whether the goal has been cancelled or not (reached) as part of the request:

```
bool cancelled
---
```

That would work fine for manual calls, although we would need to send as part of the request whether the last goal was cancelled or not. The important part is that it would work even if node A is closed and opened again, which was our initial concern. However, if the service server (node B) is closed and opened again, the goals will be reset. Without modifying the action server, the only way to save the goals since the beggining of the simulation regardless of whether node A or B have been closed, is through the *parameter server*.


### Node C (robot_status.cpp)

This node reads information published by node A to print different stats about the robot.

#### **Argument parsing**

Node C only takes as argument the rate at which it prints information:

```cpp
// Check arguments for rate:
ros::Rate *r;

if (argc == 2) {
    r = new ros::Rate((double) atof(argv[1]));
} else {
    ROS_INFO("Wrong number of arguments: (Rate).");
    exit(1);
}
```

#### **Callback function**

The whole behaviour of this node is governed by its callback function. The only thing that this node does is check for messages published by node A and use the callback function:

```cpp
while(ros::ok()) {
        
    // Check for callbacks:
    ros::spinOnce();

    // Sleep at the given rate:
    r->sleep();
}
```

* **Note:** Instead of using `ros::spin()` as in node B, we use `ros::spinOnce()` so that we can sleep manually at the given rate.

The callback function is divided in two parts: the proper callback function and an auxiliary function that only prints the stats. The callback function `get_status` takes the contents of the custom message and saves it in gloabal variables:

```cpp
float x, y, vel_x, vel_y, goal_x = 0, goal_y = 0;
int c = 0;
float acc_vx = 0, acc_vy = 0;

void get_status(const assignment_rt1_2::RobotStatus::ConstPtr &msg) {
    // Check if goal changed:
    float eps = 1e-3; // Threshold for the goal change
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
```

The reason why we also send the goal position in the custom message is to be able to detect if the goal has changed and reset the stats, in this case, the average velocity must be reset. Of course, another reason is that we need access to the goal position to compute the distance from the robot to the goal. This way, the node can be run continuously. 

Another aspect is that instead of saving every previous velocity in an array, we just save their absolute cumulative sum (`acc_vx`, `acc_vy`) and the number of callbacks (`c`) so that the average speed in either component can be computed as `acc` / `c`.

The function `print_msg` that is called at the end of the callback function is the one that computes and prints all the information:

```cpp
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
```

An interesting fact is that regardless of where the goal is, more than 90% of the speed is in the *x* direction, even if the path is a straight line parallel to the Y-axis. The only reasonable explanation for this behaviour is that the information published in the topic `"/odom"` regarding the velocity of the robot (`Twist`) is given with respect to the local frame of reference of the robot. Which would make sense since the robot always moves forward and rotates about its axis to make a turn.

### Launch file

As stated in the section [How to run the program](#how-to-run-the-program), we provide the launch file `assignment.launch` in the *launch* folder to start up the whole simulation, including the nodes in the [base package](https://github.com/CarmineD8/assignment_2_2022).

```xml
<launch>
    <include file="$(find assignment_2_2022)/launch/assignment1.launch" />
    <arg name="rate" default="2" />
    <arg name="goal_x" />
    <arg name="goal_y" />
    <param name="/goals_reached" value="0" type="int"/>
    <param name="/goals_cancelled" value="0" type="int"/>
    <node pkg="assignment_rt1_2" type="set_goal_node" name="set_goal_node" output="screen" launch-prefix="terminator -x" args="$(arg rate) $(arg goal_x) $(arg goal_y)" />
    <node pkg="assignment_rt1_2" type="robot_status_node" name="robot_status_node" output="screen" launch-prefix="terminator -x" args="$(arg rate)" />
    <node pkg="assignment_rt1_2" type="goals_status_node" name="goals_status_node" output="screen" launch-prefix="terminator -x" />
</launch>
```

As we can see, the first line launches the launch file provided in the [base package](https://github.com/CarmineD8/assignment_2_2022). Then we define the arguments that the launch file takes: `rate`, `goal_x` and `goal_y`. We decided to provide a default value of 2 Hz to the `rate` so that it can be omitted. Then, we define and set the *params* `"/goals_reached"` and `"/goals_cancelled"` to zero. Finally, we just initialize each node (A, B and C) with the corresponding arguments in a new terminal using the option `launch-prefix`.

* **Note:** We use *terminator* terminals to launch the nodes, it can be installed using the command `sudo apt install terminator`. Nevertheless, the terminal used to launch the nodes can also be changed manually, for instance, "terminator -x" can be substituted by "gnome-terminal -x", "konsole -e", "xterm -e" or whatever other terminal. However, in our *WSL* system with Ubuntu 20.04, *konsole* terminal crashes nodes A and C for a reason we have not been able to determine.

## Improvements

* **Error handling:** There is very few error checking and handling in the code, which is not ideal.

* **Wrong input handling:** There are several instances in which the user provide an input to the program and we do not check or handle it properly. For instance, we do not check if the arguments passed (`rate`, `goal_x` and `goal_y`) are numbers, we just assume they are, which will produce errors when converting it from string to float in the case that they are not numbers. Error that we have not handled. The other instance of wrong input is in the [exit routine](#exit-routine) of node A. To retry the user must input the new position of the goal, numbers that we do not check whether they are numbers and would produce the same error as before.

* **Proper exiting:** Only node A has a proper exit routine in which the program finishes and returns zero. The other two nodes are run forever and can only be stopped with signals (`SIGINT`, `SIGTERM`, etc.). Right now the easiest way is to just close the terminal in which they are initialized, which automatically sends signal `SIGHUP` to terminate the node.
