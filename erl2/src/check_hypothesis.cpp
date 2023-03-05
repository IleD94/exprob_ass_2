/** @package e
*
* \file check_hypothesis.cpp
* \brief this node implements the check_hypothesis action.
*
* \author Ilenia D'Angelo
* \version 1.0
* \date 19/02/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     None
*
* Client Services: <BR>
*  check_hypotesis
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the check_hypothesis action of the rosplan.
* This node allows to know if a new hypothesis has been found, and this is perfomed
* thanks to the check hypothesis message that advertise this node. If a new consistent hypothesis has
* been found, then the plan can proceed, if instead there is no new consistent hypothesis then
* the action return false and a replanning is needed. 
*/
#include "ros/ros.h"
#include "erl2/check_hypothesis.h"
#include <unistd.h>
#include "erl2/Consistency.h"
#include "std_msgs/Bool.h"


namespace KCL_rosplan {

    CheckHypothesisInterface::CheckHypothesisInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the check_complete action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action check_hypothesis is executed. This action 
* is performed at every waypoint after the move_arm action. To know if there are new consistent hypothesis a 
* client to the complete service is implemented. If the information given by the client is true then the plan
* can go on and can proceed with the check of the consistency, if insted the information given by the client
* is false, then a replanning is needed, and the robot will continue to explore the waypoints while collecting
* new hints.
*/     
    bool CheckHypothesisInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking the hypothesis " << std::endl;
        ros::NodeHandle n;
        ros::ServiceClient check_hypothesis_client = n.serviceClient <erl2::Consistency>("check_hypotesis");  
        erl2::Consistency srv;

        check_hypothesis_client.call (srv);
        
        if (srv.response.success == true) {
        
        	return true;
        }
        
        else {
                return false;
        }
    }
}



/**
* \brief Main function of the check_hypothesis action. 
* \param None
* \return 0
*
* This is the main function of the check_hypothesis action, where the node is initialized. Moreover there is the 
* CheckHypothesisInterface to execute the real action as an action of the rosplan.
*/
    int main(int argc, char **argv) {
        ros::init(argc, argv, "check_hypothesis_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::CheckHypothesisInterface check_hypothesis(nh);
        check_hypothesis.runActionInterface();
        ros::spin();
        return 0;
    }
