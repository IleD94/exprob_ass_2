/** @package erl2
*
* \file go_home.cpp
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
*     
*
* Action Client: <BR>
*     /reaching_goal
*
* Description: <BR>
* In this node is implemented the go_home action of the rosplan.
* This node allows to go home using the Planning action of the erl2 package. 
*/



#include "erl2/go_home.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

/////////////////////////////prendere i pacchetti in action e cambiare il nome delle cartelle qui
namespace KCL_rosplan {

	GoHomeInterface::GoHomeInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool GoHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
		erl2::PlanningGoal goal;
		ac.waitForServer();
		if (msg->parameters[1].value == "myhome"){
		goal.x = 0.0;
		goal.y = 0.0;
		goal.theta = 0.0;
		}
        else {
        std::cout << "There is an error in your destination" << std::endl;
        }
		ac.sendGoal(goal);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "go_home_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::GoHomeInterface go_home(nh);
		go_home.runActionInterface();
		return 0;
	}
