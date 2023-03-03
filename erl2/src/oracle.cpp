/** @package erl2
*
* \file oracle.cpp
* \brief this node implements the oracle action.
*
* \author Ilenia D'Angelo
* \version 1.0
* \date 19/02/23
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
*     /oracle_solution
*     /accusation
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the oracle action of the rosplan.
* This node allows to know if the current complete and consistent hypothesis is the winning one or not.
* Here we compare the value of the ID in the /oracle_solution service e in the /accusation service. If it is
* the winning one, we print the killer, the weapon and the place on the dislay.
*/

#include "erl2/oracle.h"
#include <unistd.h>
#include <erl2/Oracle.h>
#include <erl2/MyHypo.h>
#include "std_msgs/Int32.h"
#include <std_msgs/String.h>

ros::Publisher pub;
std_msgs::String mymsg;

namespace KCL_rosplan {

    OracleInterface::OracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the oracle action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action oracle is executed. This action allows
* the robot its own complete and consistent hypothesis just collected is the correct one or the wrong one.
* To do that we have a client to the oracle solution service that contains the ID of the winning hypothesis,
* the client to the winhypo service which contains the ID, who, the what and the where associated to the current hypothesis.
* If the two IDs matches, then the plan is completed since the game is finished. If not a replanning is needed. 
* At the end we publish a message to the topic "cluedo_ui", in order to display the solution on the screen.
*/    
    bool OracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "Checking if the hypothesis is the correct one" << std::endl;
        
        ros::NodeHandle n1;
        ros::ServiceClient oracle_client = n1.serviceClient<erl2::Oracle>("/oracle_solution");
        erl2::Oracle oracle_srv;

        ros::NodeHandle n;
        ros::ServiceClient accusation_client = n.serviceClient<erl2::MyHypo>("/accusation");
        erl2::MyHypo myhypo_srv;

        ros::NodeHandle nh;
        pub = nh.advertise<std_msgs::String>("cluedo_ui", 10);
        std_msgs::String mymsg;

        accusation_client.call(myhypo_srv);
        oracle_client.call(oracle_srv);
        
        sleep(1.0);
        std::cout << "Tell me your accusation" << std::endl;
        mymsg.data = "Tell me your accusation";
        pub.publish(mymsg);
        sleep(1.0);
        std::cout << " Mr. Black was killed by "<< myhypo_srv.response.who << " with the " << myhypo_srv.response.what << " in the " << myhypo_srv.response.where << std::endl;
        mymsg.data = "Mr. Black was killed by "+ myhypo_srv.response.who + " with the " + myhypo_srv.response.what + " in the " + myhypo_srv.response.where;
        pub.publish(mymsg);
        sleep(1.0);
        
        if (oracle_srv.response.ID == myhypo_srv.response.ID) {
            //std::cout << "_____________________________________________________" << std::endl;
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            std::cout << "Yes, DetectiveBot you've solved the case!" << std::endl;
            mymsg.data = "Yes, DetectiveBot you've solved the case!";
            pub.publish(mymsg);
            return true; 
        }
        
        else {
            //std::cout << "_____________________________________________________" << std::endl;
            ROS_INFO("Action (%s) not performed!", msg->name.c_str());
            std::cout << "No DetectiveBot, you made a mistake. The case is not over. Find the solution!" << std::endl;
            mymsg.data = "No DetectiveBot, you made a mistake. The case is not over. Find the solution!";
            pub.publish(mymsg);
            return false;
        }
        
    }
}


/**
* \brief Main function of the oracle action. 
* \param None
* \return 0
*
* This is the main function of the oracle action, where the node is initialized. Moreover there is the 
* OracleInterface to execute the real action as an action of the rosplan.
*/  
    int main(int argc, char **argv) {
        ros::init(argc, argv, "oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::OracleInterface oracle(nh);
        oracle.runActionInterface();
        return 0;
    }
