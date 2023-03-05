# exprob_ass_2
## Introduction
This project is the second part out of three of the implementation of Cluedo Game in simulated environment. In this part we have implemented the architecture of the game and the real movement of the player, a robot Detective, on gazebo simuator. The architecture is divided in three main part: perception, not fully implemented in this part (to have a full implementation on this part, please go to the [third repository of this project](https://github.com/IleD94/exprob_ass_3)), action, fully implemented and planning, implemented using RosPlan service.
The rules of the game are simple. There was a omicide, Mr. Black was killed by someone, somewhere with some weapon. There is a detective, Detective Bot, that goes around (in gazebo envoronment) to find hints to solve the case. Hints can be found in 4 different waypoints, at two different height (1.25 or 0.75, generated randomly for every new game)
1° waypoint: (2, 0)
2° waypoint: (0, 2)
3° waypoint: (-2, 0)
4° waypoint: (0, -2)


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

### Reasoning: PDDL
In order to plan the steps executed by the robot in the quest, we used rosplan. The language of planning used is pddl (Planning Domain Definition Language.
Our domain provides 7 possible actions: initialization, leave_home, go_to_waypoint, move_arm, check_hypothesis, go_home, oracle.
Every action was implemented as a effective action in nodes C++.
The problem created has as goal winningID, this predicates is true only if the oracle action succedeed. To check pddl files please go to che [common directory in erl2 folder](https://github.com/IleD94/exprob_ass_2/tree/main/erl2/common).

### Actions: components in c++ and python
#### Component diagram
![assignment2 drawio (1)](https://user-images.githubusercontent.com/80365922/222974962-08f5ed90-4058-4150-a64a-a76b3e9c7aa5.png)
