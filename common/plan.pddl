Number of literals: 23
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
== Warning ==

The action (goto_waypoint turtlebot wp0 wp0) has a fixed duration of zero,
but also has:

* Propositional over all conditions

In this case, correct handling of the action is unclear (e.g. at what point
must the over all/at end conditions hold if there is no gap between the start
and end of the action).  If the intention is for the action to be truly
instantaneous, use a PDDL (:action rather than a (:durative-action.  For now,
however, the action has been discarded.

Other actions to have been discarded due to zero-durations include
(goto_waypoint turtlebot wp1 wp1)
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
75% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 9.000
b (7.000 | 12.237)b (6.000 | 16.046)b (4.000 | 30.592)
Resorting to best-first search
b (7.000 | 12.237)b (6.000 | 16.046)b (6.000 | 14.474)b (4.000 | 30.592)b (3.000 | 85.424)b (2.000 | 90.672);;;; Solution Found
; States evaluated: 271
; Cost: 100.672
; Time 0.36
0.000: (goto_waypoint turtlebot wp0 wp3)  [2.236]
2.237: (inspect turtlebot wp3)  [10.000]
12.238: (goto_waypoint turtlebot wp3 wp6)  [3.808]
16.047: (goto_waypoint turtlebot wp6 wp4)  [4.545]
20.592: (inspect turtlebot wp4)  [10.000]
30.593: (goto_waypoint turtlebot wp4 wp2)  [4.187]
34.780: (dock turtlebot wp2)  [2.000]
36.780: (charge turtlebot wp2)  [39.940]
76.721: (undock turtlebot wp1)  [2.000]
78.721: (goto_waypoint turtlebot wp2 wp6)  [6.703]
85.425: (goto_waypoint turtlebot wp6 wp5)  [5.248]
90.672: (inspect turtlebot wp5)  [10.000]
