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
b (17.000 | 10.000)b (16.000 | 10.000)b (15.000 | 44.987)b (14.000 | 44.987)b (12.000 | 54.988)b (11.000 | 111.033)b (10.000 | 111.033)b (8.000 | 121.034)b (7.000 | 170.076)
Resorting to best-first search
b (17.000 | 10.000)b (16.000 | 10.000)b (15.000 | 44.987)b (14.000 | 44.987)b (12.000 | 54.988)b (11.000 | 111.033)b (10.000 | 111.033)b (8.000 | 121.034)b (7.000 | 170.076)b (6.000 | 256.522)b (4.000 | 266.523)b (3.000 | 314.794)b (2.000 | 314.794);;;; Solution Found
; States evaluated: 108
; Cost: 324.795
; Time 0.06
0.000: (inspect turtlebot wp3)  [10.000]
10.001: (goto_waypoint turtlebot wp3 wp4)  [34.986]
44.988: (inspect turtlebot wp4)  [10.000]
54.989: (goto_waypoint turtlebot wp4 wp5)  [56.045]
111.034: (inspect turtlebot wp5)  [10.000]
121.035: (goto_waypoint turtlebot wp5 wp1)  [38.275]
159.311: (dock turtlebot wp1)  [1.000]
160.311: (charge turtlebot wp1)  [40.202]
200.513: (undock turtlebot wp1)  [1.000]
201.513: (goto_waypoint turtlebot wp1 wp6)  [55.009]
256.523: (inspect turtlebot wp6)  [10.000]
266.524: (goto_waypoint turtlebot wp6 wp7)  [48.270]
314.795: (inspect turtlebot wp7)  [10.000]
