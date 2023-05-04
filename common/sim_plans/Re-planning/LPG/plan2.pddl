
; Version LPG-td-1.4
; Seed 95607697
; Command line: /home/jonas/catkin_ws/src/rosplan/rosplan_planning_system/common/bin/lpg-td -o /home/jonas/catkin_ws/src/inspection/common/domain.pddl -f /home/jonas/catkin_ws/src/inspection/common/problem2.pddl -n 20 -cputime 20 -out /home/jonas/catkin_ws/src/inspection/common/lpgplan 
; Problem /home/jonas/catkin_ws/src/inspection/common/problem2.pddl
; Time 6.39
; Search time 6.39
; Parsing time 0.00
; Mutex time 0.00
; MetricValue 33.34

0.0002:   (INSPECT TURTLEBOT WP4) [10.0000]
10.0008:   (GOTO_WAYPOINT TURTLEBOT WP4 WP3) [34.9857]
44.9867:   (INSPECT TURTLEBOT WP3) [10.0000]
54.9872:   (GOTO_WAYPOINT TURTLEBOT WP3 WP1) [46.6476]
101.6351:   (DOCK TURTLEBOT WP1) [1.0000]
102.6353:   (CHARGE TURTLEBOT WP1) [24.0518]
126.6876:   (UNDOCK TURTLEBOT WP1) [1.0000]
127.6879:   (GOTO_WAYPOINT TURTLEBOT WP1 WP5) [38.2753]
165.9634:   (INSPECT TURTLEBOT WP5) [10.0000]
175.9639:   (GOTO_WAYPOINT TURTLEBOT WP5 WP6) [49.0408]
225.0049:   (INSPECT TURTLEBOT WP6) [10.0000]
235.0054:   (GOTO_WAYPOINT TURTLEBOT WP6 WP7) [48.2701]
283.2758:   (INSPECT TURTLEBOT WP7) [10.0000]


