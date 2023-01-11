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
b (7.000 | 22.237)b (6.000 | 26.046)b (4.000 | 50.592)
Resorting to best-first search
b (7.000 | 22.237)b (6.000 | 26.046)b (6.000 | 24.474)b (4.000 | 50.592)b (3.000 | 75.483)b (2.000 | 80.732);;;; Solution Found
; States evaluated: 288
; Cost: 100.732
; Time 0.32
0.000: (goto_waypoint turtlebot wp0 wp3)  [2.236]
2.237: (inspect turtlebot wp3)  [20.000]
22.238: (goto_waypoint turtlebot wp3 wp6)  [3.808]
26.047: (goto_waypoint turtlebot wp6 wp4)  [4.545]
30.592: (inspect turtlebot wp4)  [20.000]
50.593: (goto_waypoint turtlebot wp4 wp2)  [4.187]
54.780: (dock turtlebot wp2)  [2.000]
56.780: (charge turtlebot wp2)  [10.000]
66.780: (undock turtlebot wp1)  [2.000]
68.780: (goto_waypoint turtlebot wp2 wp6)  [6.703]
75.484: (goto_waypoint turtlebot wp6 wp5)  [5.248]
80.732: (inspect turtlebot wp5)  [20.000]
