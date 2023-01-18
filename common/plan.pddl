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
8% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 2.000
b (1.000 | 10.000);;;; Solution Found
; States evaluated: 3
; Cost: 10.000
; Time 0.00
0.000: (inspect turtlebot wp3)  [10.000]
