(define (domain turtlebot3)

(:requirements :strips :typing :fluents :durative-actions)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)

)

;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and 
		(at start (robot_at ?v ?from)))
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at end (visited ?to))
		(at end (robot_at ?v ?to)))
)
)