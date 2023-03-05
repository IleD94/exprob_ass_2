# exprob_ass_2
## Introduction
This project is the second part out of three of the implementation of Cluedo Game in simulated environment. In this part we have implemented the architecture of the game and the real movement of the player, a robot Detective, on gazebo simuator. The architecture is divided in three main part: perception, not fully implemented in this part (to have a full implementation on this part, please go to the [third repository of this project](https://github.com/IleD94/exprob_ass_3)), action, fully implemented and planning, implemented using RosPlan service.
The rules of the game are simple. There was a omicide, Mr. Black was killed by someone, somewhere with some weapon. There is a detective, Detective Bot, that goes around (in gazebo envoronment) to find hints to solve the case. Hints can be found in 4 different waypoints, at two different height (1.25 or 0.75, generated randomly for every new game)
1° waypoint: (2, 0)
2° waypoint: (0, 2)
3° waypoint: (-2, 0)
4° waypoint: (0, -2)
To have more information about general description of the game, please check the repositiory of the [first part](https://github.com/IleD94/exprob_ass1)

## Robot Model
The robot of the model is contained in the directory urdf. Here we can find detective_bot.gazebo and the detective_bot.xacro files, the file materials.xacro with the materials used for the robot and detective_bot.urdf, generated during the moveit setup. Our robot is a rover with a robotic arm attached to the base of the rover. It is composed by 4 links, as it is shown in the picture below

## Moveit
MoveIt is a package for ros that allows the manipulation of robots. In this project we implemented 3 possible poses for our arm:
1. zero pose (not used in the game)
2. low pose
3. high pose

### Low Pose
![image](https://user-images.githubusercontent.com/80365922/222974351-128022c2-2342-4b85-9b13-b579ded853a8.png)

### High Pose 
![image](https://user-images.githubusercontent.com/80365922/222974511-488c4f61-0809-4cb0-9054-f7acbe363991.png)

## Software architecture

### Planning: PDDL
In order to plan the steps executed by the robot in the quest, we used rosplan. The language of planning used is pddl (Planning Domain Definition Language.
Our domain provides 7 possible actions: initialization, leave_home, go_to_waypoint, move_arm, check_hypothesis, go_home, oracle.
Every action was implemented as a effective action in nodes C++.
The problem created has as goal winningID, this predicates is true only if the oracle action succedeed. To check pddl files please go to che [common directory in erl2 folder](https://github.com/IleD94/exprob_ass_2/tree/main/erl2/common).

### Actions
#### Component diagram
![assignment2 drawio (1)](https://user-images.githubusercontent.com/80365922/222974962-08f5ed90-4058-4150-a64a-a76b3e9c7aa5.png)
#### Python nodes
1. *OntologySettings*: This is a Python script that sets the default classes and individuals in the Cluedo ontology using the Armor Client API. The ontology represents the entities in the game such as suspects, weapons, and rooms, and their relationships. We defines three lists of suspects, weapons, and rooms. This script sets up the Cluedo ontology in the Armor server and adds the necessary classes and individuals for the game.
2. *go_to_point_action*: This is a Python script that handles robot navigation from a starting point to a goal point using a state machine. The script subscribes to the robot's odometry information and publishes Twist messages to control the robot's linear and angular velocity.
3. *hint_collection*: The node subscribes to a topic "/oracle_hint" and receives messages of type "ErlOracle" which contain information about hints. The node uses the ArmorClient library to interact with the ontology.
When a new hint message is received, the node checks if the hint is malformed and discards it if it is. Otherwise, the hint is added to the "HYPOTHESIS" class of the ontology, along with information about the source (ID), the key (who/what/where), and the value of the hint. The node then checks if the hypothesis is consistent or not by calling the "ontology_query" function, which queries the ontology using the ArmorClient and returns a Boolean value indicating if the hypothesis is consistent or not.The node also provides two ROS services. The first service is "check_hypotesis", which simply returns the Boolean value of the "check" global variable, which is updated by the "ontology_query" function. The second service is "accusation", which calls the "make_ind_of_class_disjoint" function to make all individuals of the classes "PERSON", "WEAPON", and "PLACE" disjoint. It then queries the ontology using the ArmorClient to retrieve the "who", "what", and "where" values of a specific hypothesis (specified by the "HP" variable), and returns these values in a "MyHypo" response.
All the information concerning the game that can be useful for the player are shown in the user_interface
4. *Plan_exec*: This is a Python script that executes a plan generated by the ROSPlan framework. It calls several ROS services provided by ROSPlan to generate a problem, plan, parse the plan, and dispatch the plan. The script then waits for feedback from the dispatch service to determine if the plan execution was successful or not. If the plan execution was not successful or the goal was not achieved, the script starts the planning process again. The inferred ontology is then saved to a new file using the armor api save_ref_with_inferences function.
5. *user_interface*: This node just implements the interface with the user. It waits for some strings published to the 'cluedi_ui' topic to dislay them on the screen.

![interface](https://user-images.githubusercontent.com/80365922/222978130-16fc5cd4-2eac-4dcd-8eb6-3d2365b4c5dd.png)


#### c++ nodes
1. *Initialization*: This node implements an action server for an initialization action in a ROS environment. The purpose of this action is to perform a complete first tour moving the arm both in the high position and then in the low position. This is done only to know the z position of the sphere in the environment, in a way that, in the following rounds, the robot already knows the position of those spheres and positions directly in the right position.
2. *check_hypothesis*: This code implements the check_hypothesis action in the rosplan framework. This action is performed at every waypoint, after the move_arm action. Its purpose is to check if there are new consistent hypotheses, using the check_hypothesis service. If the service returns true, then the plan can proceed with the go_home action. Otherwise, if the service returns false, a replanning is needed.
3. *go_home*: This code defines a ROS node for the "go_home" action of a robot. This action is performed after the "check_hypothesis" action and allows the robot to go back to its home location to check if the hypothesis is the right one.
4. *leave_home*: This code defines a ROS node for the "leave_home" action of a robot. This action is performed after the "initialization" action and  the "oracle" action, if the hypothesis was not the winning one. It allows the robot to leave its home location to search for hints in the waypoints.
5. *Go_to_waypoint*: This node implements the movements from a waypoints to another
6. *more_arm*: The move_arm action allows the robot to move its arm to either the high or low pose depending on the value retrieved by the parameter server previously set in the initialization action.
![low](https://user-images.githubusercontent.com/80365922/222978148-e11c5fd9-2010-4e8f-bbbc-10bc6fa4f74e.png)

![high](https://user-images.githubusercontent.com/80365922/222978151-33fdaf9f-922f-4c6f-bd12-9caf68860095.png)

7. *Oracle*: This code defines the implementation of the oracle action for a ROSPlan system. This action is responsible for checking if the current hypothesis proposed by the system is the correct one. If the IDs of the winning hypothesis and the current hypothesis match, the action is considered completed and a message is published to the "cluedo_ui" topic to inform the user that they have solved the case. Otherwise, the action is considered not performed and a message is published to the "cluedo_ui" topic to inform the user that they made a mistake.
8. *Simulation* : This node was gently provided by prof.Carmine Recchiuto. It manages the hints generation and the choose of the winning ID.

![interface_solved](https://user-images.githubusercontent.com/80365922/222978182-42ef5400-9d0f-4958-9e2d-2448b70c6375.png)


## How to install and run:
To install the package please copy it with this command:
```
git clone https://github.com/IleD94/exprob_ass_2
```
Then move the repository cluedo_moveit to another directory and build it with:
```
catkin_make
```
And after that build also the erl2 package, with the command:
```
catkin_make --only-pkg-with-deps erl2
```
To run the code you have to launch this files in the order below, to avoid any conflict with armor package:

```
roslaunch erl2 my_scripts.launch
```
```
roslaunch erl2 myass_gaz.launch 1>/dev/null 2>/dev/nul
```
```
roslaunch erl2 rosplan_launch.launch 
```
```
rosrun erl2 plan_exec.py 
```

## Video


https://user-images.githubusercontent.com/80365922/222978053-acfe352c-84e3-49fa-a0be-3a4d62efe382.mp4


## Contacts
Author: Ilenia D'Angelo,

email: ileniadangelo94@gmail.com
