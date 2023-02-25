#!/usr/bin/env python
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

#pub = rospy.Publisher("check_hypotesis", Bool, queue_size=1)
acquired = False

client = ArmorClient("cluedo", "ontology")
#HP = None
ID = None
ID_list= []
check = False
go = False
def user_interface(msg):

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
    global HP #forse va tolto
    
    HP = "HP"+str(ID)
    #myID = rospy.get_param (HP) #CONSISTENT
    client.manipulation.add_ind_to_class(HP, "HYPOTHESIS")
    client.manipulation.add_dataprop_to_ind("hasID", "HP"+str(ID), "STRING", str(ID))
    client.manipulation.add_objectprop_to_ind(key, HP, item)
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()
    return (HP)


def hint_callback(msg):
    global ID, key, value, acquired
    ID = msg.ID
    key = msg.key
    value = msg.value
    acquired = True 

def ontology_query (HP):
    inconsistent_list = client.query.ind_b2_class("INCONSISTENT")
    complete_list = client.query.ind_b2_class("COMPLETED")
    inconsistent_str = str(inconsistent_list)
    complete_str = str(complete_list)
    print(HP)
    if (complete_str.find (str(HP)) != -1):
        if (inconsistent_str.find (HP) == -1):
            user_interface ('The ' + HP + 'is CONSISTENT')
            #ID_list.append(ID)
            return True
        else:
            user_interface ('The ' + HP + ' is INCONSISTENT')
            time.sleep(1)
            # msg_consistency = Bool()
            # msg_consistency.data = False
            # pub.publish (msg_consistency)
            return False
    else:
            user_interface ('The ' + HP + ' is INCOMPLETE')
            # msg_consistency = Bool()
            # msg_consistency.data = False
            # pub.publish (msg_consistency)
            return False

def from_url_to_my_item (url):
    my_string = str (url)
    my_string = my_string.replace ('<http://www.emarolab.it/cluedo-ontology#', '')
    my_item = my_string.replace ('>', '')
    return my_item

def accusation_maker (req):
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
    user_interface ("ENTRO NEL MAKER!!!!")
    return res

def check_hypo (req):
    global check
    res = ConsistencyResponse()
    res.success = check
    return res
    
def check_callback (data):
    global go
    go = True

def main ():
    global myhypo_pub, acquired, HP, check
    # Inizializza il nodo "publisher_py"
    rospy.init_node('hints_collection', anonymous = True)
    hint_sub = rospy.Subscriber('/oracle_hint', ErlOracle, hint_callback)
    service_consistency = rospy.Service("check_hypotesis", Consistency, check_hypo)
    myhypo_srv = rospy.Service('accusation', MyHypo, accusation_maker)
    check_sub = rospy.Subscriber("check", Bool, check_callback)
    rate = rospy.Rate(1)
    # Codice del publisher
    while not rospy.is_shutdown():
        #if go:
		# if a new hint has been collected
            if acquired == True:
                # if the hint is a malformed hint the robot will recognise and discard it
                if key == '' or key == 'when' or value == '' or value == '-1':
                    
                    user_interface('Malformed hint, the robot will discard this') #mettere una user interface, come nel primo assignment
                # instead if the hint is not a malformed one
                else:
                    #check = False;
                    user_interface('Hint collected: {}, {}, {}'.format(ID, key, value))   
                    # uploading the hint in the ontology
                    HP = add_hypothesis (ID, value, key)
                    # check if completed hypotheses have been loaded in the ontology 
                    query_res=ontology_query(HP)
                    time.sleep (1)
                    if query_res:
                        user_interface ('CI SONOOOOOOOOOOOOOOOOOOO')
                        if ID in ID_list:
                            check = False
                        else:
                            check = True
                            ID_list.append(ID)
                            time.sleep (15)
                            #rate.sleep ()

                    else:
                            check = False


                        # if there is only one element in the COMPLETED class

                rospy.sleep(10)
                acquired = False
                rate.sleep()

            # if there are not new hint collected    
            else:
                # the complete variable is set to false
                #check = False
                user_interface ('Sono qui')
                rate.sleep()
        # else:
        #     rate.sleep()  



if __name__ == '__main__':
    main ()
