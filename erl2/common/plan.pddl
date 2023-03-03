Number of literals: 15
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 5.000
b (4.000 | 10.000)b (3.000 | 16.001)b (2.000 | 21.002)b (1.000 | 26.003);;;; Solution Found
; States evaluated: 6
; Cost: 33.004
; Time 0.00
0.000: (goto_waypoint wp1 wp2)  [10.000]
10.001: (move_arm wp2)  [6.000]
16.002: (check_hypothesis wp2)  [5.000]
16.003: (go_home wp2 myhome)  [10.000]
26.004: (oracle myhome wp2)  [7.000]
