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
Initial heuristic = 9.000
b (8.000 | 1.000)b (7.000 | 56.010)b (6.000 | 66.011)b (5.000 | 98.570)b (4.000 | 142.584)b (3.000 | 168.079)b (2.000 | 168.079);;;; Solution Found
; States evaluated: 17
; Cost: 36.758
; Time 0.02
0.000: (undock turtlebot wp1)  [1.000]
1.001: (goto_waypoint turtlebot wp1 wp6)  [55.009]
56.011: (inspect turtlebot wp6)  [10.000]
66.012: (goto_waypoint turtlebot wp6 wp0)  [32.558]
98.570: (dock turtlebot wp0)  [1.000]
99.570: (charge turtlebot wp0)  [42.014]
141.584: (undock turtlebot wp0)  [1.000]
142.584: (goto_waypoint turtlebot wp0 wp7)  [25.495]
168.080: (inspect turtlebot wp7)  [10.000]
