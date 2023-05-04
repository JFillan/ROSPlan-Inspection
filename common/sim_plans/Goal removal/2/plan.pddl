Number of literals: 18
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
Pruning (goto_waypoint turtlebot wp0 wp0) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp1 wp1) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp2 wp2) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp3 wp3) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp4 wp4) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp5 wp5) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp6 wp6) - never appeared in initial RPG
Pruning (goto_waypoint turtlebot wp7 wp7) - never appeared in initial RPG
5% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 18.000
b (17.000 | 10.000)b (16.000 | 10.000)b (15.000 | 44.987)b (14.000 | 44.987)b (12.000 | 54.988)b (11.000 | 139.894)b (10.000 | 149.895)b (9.000 | 220.075)b (8.000 | 263.892)b (7.000 | 296.450)b (6.000 | 296.450)b (4.000 | 306.451)b (3.000 | 354.722)b (2.000 | 354.722);;;; Solution Found
; States evaluated: 35
; Cost: 11.614
; Time 0.06
0.000: (inspect turtlebot wp4)  [10.000]
10.001: (goto_waypoint turtlebot wp4 wp3)  [34.986]
44.988: (inspect turtlebot wp3)  [10.000]
54.989: (goto_waypoint turtlebot wp3 wp5)  [84.906]
139.895: (inspect turtlebot wp5)  [10.000]
149.897: (goto_waypoint turtlebot wp5 wp0)  [70.178]
220.075: (dock turtlebot wp0)  [1.000]
221.075: (charge turtlebot wp0)  [41.817]
262.892: (undock turtlebot wp0)  [1.000]
263.892: (goto_waypoint turtlebot wp0 wp6)  [32.558]
296.451: (inspect turtlebot wp6)  [10.000]
306.452: (goto_waypoint turtlebot wp6 wp7)  [48.270]
354.723: (inspect turtlebot wp7)  [10.000]
