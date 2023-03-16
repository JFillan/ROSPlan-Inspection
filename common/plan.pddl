
; Version LPG-td-1.4
; Seed 13399540
; Command line: /home/jonas/catkin_ws/src/rosplan/rosplan_planning_system/common/bin/lpg-td -o /home/jonas/catkin_ws/src/inspection/common/domain.pddl -f /home/jonas/catkin_ws/src/inspection/common/problem2.pddl -quality -out /home/jonas/catkin_ws/src/inspection/common/lpgplan 
; Problem /home/jonas/catkin_ws/src/inspection/common/problem2.pddl
; Time 0.76
; Plan generation time 0.49
; Search time 0.49
; Parsing time 0.00
; Mutex time 0.00
; MetricValue 32.66

0.0003:   (GOTO_WAYPOINT TURTLEBOT WP0 WP2) [29.4109]
29.4114:   (INSPECT TURTLEBOT WP2) [10.0000]
39.4117:   (GOTO_WAYPOINT TURTLEBOT WP2 WP7) [41.4367]
80.8486:   (INSPECT TURTLEBOT WP7) [10.0000]
90.8488:   (GOTO_WAYPOINT TURTLEBOT WP7 WP0) [25.4951]
116.3442:   (GOTO_WAYPOINT TURTLEBOT WP0 WP6) [32.5576]
148.9021:   (INSPECT TURTLEBOT WP6) [10.0000]
158.9023:   (GOTO_WAYPOINT TURTLEBOT WP6 WP1) [55.0091]
213.9117:   (DOCK TURTLEBOT WP1) [1.0000]
214.9119:   (CHARGE TURTLEBOT WP1) [32.0864]
246.9986:   (UNDOCK TURTLEBOT WP1) [1.0000]
247.9988:   (GOTO_WAYPOINT TURTLEBOT WP1 WP4) [22.8035]
270.8026:   (INSPECT TURTLEBOT WP4) [10.0000]
280.8028:   (GOTO_WAYPOINT TURTLEBOT WP4 WP3) [34.9857]
315.7888:   (INSPECT TURTLEBOT WP3) [10.0000]
325.7890:   (GOTO_WAYPOINT TURTLEBOT WP3 WP5) [84.9058]
410.6950:   (INSPECT TURTLEBOT WP5) [10.0000]


