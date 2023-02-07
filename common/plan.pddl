Number of literals: 20
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
== Warning ==

The action (goto_waypoint turtlebot wp0 wp0) has a fixed duration of zero,
but also has:

* Propositional over all conditions
* Numeric at end conditions

In this case, correct handling of the action is unclear (e.g. at what point
must the over all/at end conditions hold if there is no gap between the start
and end of the action).  If the intention is for the action to be truly
instantaneous, use a PDDL (:action rather than a (:durative-action.  For now,
however, the action has been discarded.

Other actions to have been discarded due to zero-durations include
(goto_waypoint turtlebot wp1 wp1)
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
12% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 10.000)b (8.000 | 10.000)b (6.000 | 42.361)b (5.000 | 64.721)b (4.000 | 193.252)b (3.000 | 266.629)b (2.000 | 266.629);;;; Solution Found
; States evaluated: 30
; Cost: 276.629
; Time 0.02
0.000: (inspect turtlebot wp0)  [10.000]
10.000: (goto_waypoint turtlebot wp0 wp3)  [22.361]
32.361: (inspect turtlebot wp3)  [10.000]
42.361: (goto_waypoint turtlebot wp3 wp0)  [22.361]
64.722: (goto_waypoint turtlebot wp0 wp1)  [44.822]
109.544: (dock turtlebot wp1)  [1.000]
110.544: (charge turtlebot wp1)  [36.886]
147.430: (undock turtlebot wp1)  [1.000]
148.430: (goto_waypoint turtlebot wp1 wp0)  [44.822]
193.253: (goto_waypoint turtlebot wp0 wp5)  [73.376]
266.629: (inspect turtlebot wp5)  [10.000]
