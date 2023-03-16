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
Initial heuristic = 24.000
b (23.000 | 29.411)b (22.000 | 29.411)b (20.000 | 39.412)b (19.000 | 91.153)b (18.000 | 91.153)b (16.000 | 101.154)b (15.000 | 136.140)b (14.000 | 136.140)b (12.000 | 146.141)b (11.000 | 202.187)b (10.000 | 212.188)b (9.000 | 282.367)b (8.000 | 326.721)b (7.000 | 359.279)b (6.000 | 359.279)b (4.000 | 369.280)b (3.000 | 417.551)b (2.000 | 417.551);;;; Solution Found
; States evaluated: 46
; Cost: 0.000
; Time 0.08
0.000: (goto_waypoint turtlebot wp0 wp2)  [29.411]
29.412: (inspect turtlebot wp2)  [10.000]
39.413: (goto_waypoint turtlebot wp2 wp3)  [51.740]
91.154: (inspect turtlebot wp3)  [10.000]
101.155: (goto_waypoint turtlebot wp3 wp4)  [34.986]
136.141: (inspect turtlebot wp4)  [10.000]
146.142: (goto_waypoint turtlebot wp4 wp5)  [56.045]
202.188: (inspect turtlebot wp5)  [10.000]
212.189: (goto_waypoint turtlebot wp5 wp0)  [70.178]
282.367: (dock turtlebot wp0)  [1.000]
283.367: (charge turtlebot wp0)  [42.354]
325.721: (undock turtlebot wp0)  [1.000]
326.721: (goto_waypoint turtlebot wp0 wp6)  [32.558]
359.280: (inspect turtlebot wp6)  [10.000]
369.281: (goto_waypoint turtlebot wp6 wp7)  [48.270]
417.552: (inspect turtlebot wp7)  [10.000]
