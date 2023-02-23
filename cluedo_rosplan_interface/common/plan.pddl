Number of literals: 16
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 5.000
b (4.000 | 60.000)b (3.000 | 76.002)b (2.000 | 81.003)b (1.000 | 86.004);;;; Solution Found
; States evaluated: 7
; Cost: 93.005
; Time 0.01
0.000: (initialization myhome)  [60.000]
60.001: (leave_home wp0 myhome)  [10.000]
70.002: (move_arm wp0)  [6.000]
76.003: (check_hypothesis wp0)  [5.000]
76.004: (go_home wp0 myhome)  [10.000]
86.005: (oracle myhome wp0)  [7.000]
