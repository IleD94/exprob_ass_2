#!/usr/bin/env python

""" 
@package cluedo, hints_collection.
This node, 'hints_collection', collects and checks all the hints received.
"""

import rospy
from std_msgs.msg import Bool
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient
from erl2.srv import MyHypo, MyHypoResponse
from erl2.srv import Consistency, ConsistencyResponse
from erl2.msg import ErlOracle
import time
from std_msgs.msg import String

acquired = False

client = ArmorClient("cluedo", "ontology")
ID = None
ID_list= []
check = False
go = False


def user_interface(msg):
    """
    /brief this function calls the publisher of the topic 'cluedo_ui', of type String and publishes it.
    @param msg: String
    @return None
    """
    pub = rospy.Publisher('cluedo_ui', String, queue_size=10) 
    time.sleep(1)
    try:
        rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException:
        pass

def make_ind_of_class_disjoint (class_name):
    """
    /brief Disjoint all individuals of a class in an ontology by comunication with the armor server.

    Args:
        ind_name (str): individual to be disjointed to the class.
        class_name (str): individual will be disjointed to this class. It will be created a new class if it does not exist.

    Returns:
        bool: True if ontology is consistent, else False

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        res = client.call('DISJOINT', 'IND', 'CLASS', [class_name])

    except rospy.ServiceException as e:
        raise ArmorServiceCallError(
            "Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if res.success:
        return res.is_consistent
    else:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)



def add_hypothesis (ID, item, key):
    """
    /brief this function add items in the class HYPOTHESIS in the cluedo ontology
    and add an individual to the object property who, where and what, depending on 
    the kind of hint that it has received from the hint_generator. At the end it
    saves the changes and synchronize the reasoner using armor api.
    @param ID: uint32 with the number of the source
    @param item: the hint to add in the cluedo ontology as where, who or what instance 
    """
    global HP 
    HP = "HP"+str(ID)
    #myID = rospy.get_param (HP) #CONSISTENT
    client.manipulation.add_ind_to_class(HP, "HYPOTHESIS")
    client.manipulation.add_dataprop_to_ind("hasID", "HP"+str(ID), "STRING", str(ID))
    client.manipulation.add_objectprop_to_ind(key, HP, item)
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()
    return (HP)


def hint_callback(msg):
    """
    /brief this is the callback of the subscriber to the /oracle_hint topic. The message is a
    custom message ErlOracle. 
    @param: msg
    @return: None
    
    This is a callback of the suscriber hint_sub. Here we collect and store as global variable
    the hint, in the form of id, key and value. Here we put as true also the flag acquired, that
    is used to know if the hint has been acquired.
    """
    global ID, key, value, acquired
    ID = msg.ID
    key = msg.key
    value = msg.value
    acquired = True 

def ontology_query (HP):
    """
    /brief this is the function to check, asking to the armor service, if the hypothesis is 
    consistent, inconsistent or incomplete.
    @param: HP
    @return: bool
    
    This function asks to armor service about the consistency of every hypothesis and 
    sends to the user interface the result of the query as a string, then it returns a 
    boolean value, true if it is consistent, false in every other case.
    """
    inconsistent_list = client.query.ind_b2_class("INCONSISTENT")
    complete_list = client.query.ind_b2_class("COMPLETED")
    inconsistent_str = str(inconsistent_list)
    complete_str = str(complete_list)
    print(HP)
    if (complete_str.find (str(HP)) != -1):
        if (inconsistent_str.find (HP) == -1):
            user_interface ('The ' + HP + ' is CONSISTENT')
            if ID in ID_list:
                user_interface ("But we've already checked and it is not the winning one")
            return True
        else:
            user_interface ('The ' + HP + ' is INCONSISTENT')
            time.sleep(1)
            return False
    else:
            user_interface ('The ' + HP + ' is INCOMPLETE')
            return False

def from_url_to_my_item (url):
    """
    /brief this a function to transform the result of the armor service, that is a url,
    to a string, with only the individual of the class. 
    @param: url, the url from the ontology
    @return: my_item, the individual from the ontology as string
    
    This function replace url, parenthesis and major symbol with empty space, In rder to have
    as a string only the individual of the class.
    """
    my_string = str (url)
    my_string = my_string.replace ('<http://www.emarolab.it/cluedo-ontology#', '')
    my_item = my_string.replace ('>', '')
    my_item = my_item.replace ('[', '')
    my_item = my_item.replace (']', '')
    return my_item

def accusation_maker (req):
    """
    /brief this is the function of the service 'accusation', of type MyHypo. In order to take
    the hint, in the form of: who, where, what, of the hypothesis that we want to check to the 
    oracle.
    @param: req, type none 
    @return: res, type: ID (int32), who (string), what, (string), where (string)
    
    This function call the function make_ind_of_class_disjoint, to make every individual of classes
    PERSON, WEAPON, PLACE disjointed. Then it calls the armor api, query.objectprop_b2_ind, to
    get who, what and where of a specific hypothes and it calls the function from_url_to_my_item
    to get from the armor_service only the individual of the class.
    """
    make_ind_of_class_disjoint ('PERSON')
    make_ind_of_class_disjoint ('WEAPON')
    make_ind_of_class_disjoint ('PLACE')
    who_url = client.query.objectprop_b2_ind ('who', HP)
    who = from_url_to_my_item (who_url)
    what_url = client.query.objectprop_b2_ind ('what', HP)
    what = from_url_to_my_item (what_url)
    where_url = client.query.objectprop_b2_ind ('where', HP)
    where = from_url_to_my_item (where_url)
    res = MyHypoResponse ()
    string_ID = HP.replace('HP','')
    res.ID = int (string_ID)
    res.who = who
    res.what = what
    res.where = where
    return res

def check_hypo (req):
    """
    /brief this is the function of the service "check_hypotesis", of type Consistency. In order to
    check consistency and completeness.
    @param: req, type none 
    @return: res, type: success (Bool) 

    Here the boolean of return is stored in the check global variable.
    """
    global check
    res = ConsistencyResponse()
    res.success = check
    return res


def main ():
    """
    /brief this is the main function. Here we declare every service and subscriber used and it checks
    if the hint is malformed, so it needs to be discarded or if it can be collected. Once collected the hint
    it controlled if the hypothesis is consistent, using the proper functions. Here is used a user_interface to
    show on the screen the result of the quest.

    @param: None
    @return: None    
    """
    global acquired, HP, check
    rospy.init_node('hints_collection', anonymous = True)
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    service_consistency = rospy.Service("check_hypotesis", Consistency, check_hypo)
    myhypo_srv = rospy.Service('accusation', MyHypo, accusation_maker)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
            if acquired == True:
                # if the hint is a malformed hint the robot will recognise and discard it
                if key == '' or key == 'when' or value == '' or value == '-1':
                    
                    user_interface('Malformed hint, the robot will discard this') 
                # instead if the hint is not a malformed one
                else:
                    user_interface('Hint collected: {}, {}, {}'.format(ID, key, value))   
                    # uploading the hint in the ontology
                    HP = add_hypothesis (ID, value, key)
                    # check if completed hypotheses have been loaded in the ontology 
                    query_res=ontology_query(HP)
                    time.sleep (1)
                    if query_res:
                        if ID in ID_list:
                            check = False
                        else:
                            check = True
                            ID_list.append(ID)
                            time.sleep (10)

                    else:
                            check = False

                rospy.sleep(10)
                acquired = False
                rate.sleep()

            # if there are not new hint collected    
            else:
                check = False
                rate.sleep()

if __name__ == '__main__':
    main ()
