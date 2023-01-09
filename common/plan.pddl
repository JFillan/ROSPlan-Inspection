Warnings encountered when parsing Domain/Problem File

Errors: 0, warnings: 1
/home/jonas/catkin_ws/src/inspection/common/domain.pddl: line: 78: Warning: Re-declaration of symbol in same scope: charge
Number of literals: 15
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
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
All the ground actions in this problem are compression-safe
Initial heuristic = 3.000
b (2.000 | 4.473)b (1.000 | 17.915);;;; Solution Found
; States evaluated: 9
; Cost: 25.254
; Time 0.00
0.000: (goto_waypoint turtlebot wp0 wp3)  [2.236]
2.237: (goto_waypoint turtlebot wp3 wp0)  [2.236]
4.474: (goto_waypoint turtlebot wp0 wp4)  [6.720]
11.195: (goto_waypoint turtlebot wp4 wp0)  [6.720]
17.916: (goto_waypoint turtlebot wp0 wp5)  [7.338]
