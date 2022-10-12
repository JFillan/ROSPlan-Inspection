(define (domain turtlebot3)

(:requirements :strips :typing :fluents)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
)

;; Move to any waypoint, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and (robot_at ?v ?from))
	:effect (and
		(not (robot_at ?v ?from))
		(visited ?to)
		(robot_at ?v ?to))
)
)