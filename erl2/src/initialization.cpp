/** @package erl2
*
* \file initialization.cpp
* \brief this node implements the initialization action.
*
* \author Ilenia D'Angelo
* \version 1.0
* \date 13/02/2023
*
* \details
*
* Subscribes to: <BR>
*     /oracle_hint
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     None
*
* Client Services: <BR>
*     None
*
* Action Client: <BR>
*     /reaching_goal
*
* Description: <BR>
* In this node is implemented the first action of the plan when the simulation starts.
* This node simply perform a complete first tour moving the arm both in the high position and then in the low
* position. This is done only to know the z position of the sphere in the environment, in a way that, in the
* following rounds the robot already knows the position of those spheres and positions directly in the right
* position. 
*/

#include "erl2/initialization.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>
#include <string>

#define PI 3.14159

// subscriber of the topic oracle_hint
ros::Subscriber hint_sub;

//global variable to know if a hint is taken
bool gotta_hint = false;

/**
* \brief Callback function of the subscriber to the /oracle_hint topic. 
* \param msg: message that contains the hint
* \return None
*
* This is the callback function of the subscriber to the /oracle_hint topic. In this case the 
* message is not collected but a variable is set to true when a message is recieved. 
*/
void hint_callback (const erl2::ErlOracle::ConstPtr& msg){

    gotta_hint = true;
}

// function to move the robot
void move_to(double xpos, double ypos, double orientation);
// function to go in exploration to find the z position of the hints, it moves the robot and the arm.
void look_around (double xpos, double ypos, double orientation, std::string wp, moveit::planning_interface::MoveGroupInterface& group);

// function to adjust the robotic arm in the high pose
int high_pose();
// function to adjust the robotic arm in the low pose
int low_pose();

//funtion to storage the z position of the robot
double z_storage (std::string wp);


// markers position
double z_wp0 = 0.0;
double z_wp1 = 0.0;
double z_wp2 = 0.0;
double z_wp3 = 0.0;



namespace KCL_rosplan {

    InitializationInterface::InitializationInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
    
    
/**
* \brief Callback of the initialization action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action initialization is executed. In particular
* this action is only execute once at the beginning of the simulation, with the purpose of store in the 
* parameter server the z position of the hint spheres in the environment.
*/
    bool InitializationInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        std::cout << "The robot is powering on" << std::endl;

        std::cout << "Initializing ARMOR" << std::endl;
        sleep(3);
        std::cout << "Loading the ontology" << std::endl;
        sleep(1);
        std::cout << "Uploading the TBox" << std::endl;
        sleep(1);
        std::cout << "Disjoint the individuals of all classes" << std::endl;
        sleep(1);
        std::cout << "Start visiting all the waypoints" << std::endl;
        
        moveit::planning_interface::MoveGroupInterface group("arm");
        group.setEndEffectorLink("cluedo_link");
        group.setPoseReferenceFrame("base_link");
        group.setPlannerId("RRTstar");
        group.setNumPlanningAttempts(10);
        group.setPlanningTime(10.0);
        group.allowReplanning(true);
        group.setGoalJointTolerance(0.0001);
        group.setGoalPositionTolerance(0.0001);
        group.setGoalOrientationTolerance(0.001);
        group.setNamedTarget("low");
        group.move();
        sleep(3.0);
        
        
        
        look_around (2.5, 0.0, 0.0, "wp0", group);

        look_around (0.0, 2.5, PI/2, "wp1", group);

        look_around (-2.5, 0.0, PI, "wp2", group);

        look_around (0.0, -2.5, -PI/2, "wp3", group);
        
        // return home
        move_to(0.0, 0.0, 0.0);
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}


/**
* \brief Main function of the initialization action. 
* \param None
* \return 0
*
* This is the main function of the initilization_action, where the node is initialized and the subscriber of the
* oracle_hint is implemented. The InitializationInterface executes the real action as an action
* of the rosplan.
*/
    int main(int argc, char **argv) {
    
        ros::init(argc, argv, "initialization_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");

        //subscriber to the /oracle_hint topic
        hint_sub = nh.subscribe("/oracle_hint", 10, hint_callback);

        
        KCL_rosplan::InitializationInterface initialization(nh);
        initialization.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        //ros::spin ();
        sleep(1.0);
        return 0;
    }



/**
* \brief Function to make the robot move in the enviornment. 
* \param xpos: x position in the environment
* \param ypos: y position in the environment
* \param orientation: desired orientation in the environment
* \return None
*
* This function implement an action client in order to make the robot move within the 
* environment that allows also to stop immediatly the behavior of the robot when this
* node is stopped. 
*/ 
void move_to(double xpos, double ypos, double orientation) {

    actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
    erl2::PlanningGoal goal;
    ac.waitForServer();
    goal.x = xpos;
    goal.y = ypos;
    goal.theta = orientation;
    ac.sendGoal(goal);
    ac.waitForResult();
    
}

/**
* \brief Function to make the robot move in the enviornment. 
* \param wp: string with the name of the waypoint
* \return double: the z position of the waypoint
*
* This function storage the z position of the hint of a specific waypoint 
*/ 
double z_storage(std::string wp) {
    double z_wp;
    if (gotta_hint == true){
            z_wp = 0.75;
        }
        // otherwise it will be collected in the low position
    else {
            z_wp = 1.25;
        }
        
        // set the z position of the marker with a parameter server
        ros::param::set(wp, z_wp);
        
        return z_wp;
}

/**
* \brief Function to make the robot move in the enviornment. 
* \param xpos: x position of the waypoint
* \param ypos: y position of the waypoint
* \param orientation: orientation of the waypoint
* \param wp: string with the name of the waypoint
* \param group: pointer to te group ("arm") definend in move it service
* \return None
*
* This function perform the exploration of the environment and call the function to storage the z postion 
*/ 

void look_around (double xpos, double ypos, double orientation, std::string wp, moveit::planning_interface::MoveGroupInterface& group) {
    // move to the first waypoint 
    move_to(xpos, ypos, orientation);
    
    double z_wp;
    z_wp = z_storage (wp);

    if (z_wp == 1.25) {

    //  high pose of the arm
    group.setNamedTarget("high");
    group.move();
    sleep(3.0);

    //come back in the most aerodynamic position
    group.setNamedTarget("low");
    group.move();
    sleep(3.0);
    }
    
    gotta_hint = false;
    
    std::cout << "The marker at " << wp << " is at: " << z_wp << std::endl;
    
}
