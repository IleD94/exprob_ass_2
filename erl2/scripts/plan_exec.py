#!/usr/bin/env python

## @package erl2
#
# \file plan_exec.py
# \brief script that menages the execution of the rosplan.
#
# \author Ilenia D'Angelo
# \version 3.0
# \date 23/02/2023
# \details
#
# Subscribes to: <BR>
#     None
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     None
#
# Client Services: <BR>
#     armor_interface_srv
#     /rosplan_problem_interface/problem_generation_server
#     /rosplan_planner_interface/planning_server
#     /rosplan_parsing_interface/parse_plan
#     /rosplan_plan_dispatcher/dispatch_plan
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node all rosplan services are called in order to execute the current plan, generated
#     automatically by rosplan services and poptf planner from the domain and the problem files provided. 
#     Services are called in loop, for every fail of the previous plan there is an automatic replanning. 

import rospy
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from rosplan_knowledge_msgs.srv import *
from rosplan_dispatch_msgs.srv import *
from armor_api.armor_client import ArmorClient
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue

upd_knowledge = None
rosplan_success = False
rosplan_goal = False




##
# \brief Main function of the node in which the node is initialized and all the rosplan services are called.
# \param: None
# \return: None
#
# This is the main function of the node, where the node is initialized and all the rosplan services are called.
# Here replanning is called after a fail.

def main():

    global rosplan_success, rosplan_goal, client
    rospy.init_node('plan_execution')
    
    # calling all the rosplan services 
    print ("Calling all rosplan services ..")
    # problem interface service
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_interface = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    # planning interface service
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning_interface = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    # parsing interface service
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing_interface = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    # dispatch plan
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    plan_dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    
    #call of the class ArmorClient from armor_api
    client = ArmorClient("cluedo", "ontology")

    print("Start planning")
    
    rosplan_success = False
    rosplan_goal = False
    
    while (rosplan_success == False or rosplan_goal == False):
        
        print('Problem interface service')
        problem_interface()

        print('Planning interface service')
        planning_interface()

        print('Parsing interface service')
        parsing_interface()

        print('Plan dispatcher')
        feedback = plan_dispatcher()
        print(feedback)
        
        rosplan_success = feedback.success
        rosplan_goal = feedback.goal_achieved
        
        print('Replanning')
    
##
# \brief It saves the changes on a new ontology file, calling the armor api save_ref_with_inferences of the armor client.
# \param: None
# \return: None
#
# Here we save the new inferred ontology on the Desktop folder.

    client.utils.save_ref_with_inferences("/root/Desktop/inferred_cluedo.owl")
    print('The plan has finished!')


if __name__ == '__main__':
    main()
