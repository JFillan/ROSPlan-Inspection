(define (domain turtlebot3)

(:requirements :strips :typing :fluents :durative-actions)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(dock_at ?wp - waypoint)
	(battery_charged ?v - robot)
)

(:functions
	(distance ?wp1 ?wp2 - waypoint) 
	(speed ?v - robot)
)


;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration (/ (distance ?from ?to) 
							  (speed ?v)))
	:condition (and 
		(at start (robot_at ?v ?from))
		(over all (battery_charged ?v)))
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at end (visited ?to))
		(at end (robot_at ?v ?to)))
)

(:durative-action dock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 30)
	:condition (and
		(over all (dock_at ?wp))
		(at start (robot_at ?v ?wp))
		(at start (undocked ?v)))
	:effect (and
		(at end (docked ?v))
		(at start (not (undocked ?v))))
)

(:durative-action undock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(over all (dock_at ?wp))
		(at start (docked ?v)))
	:effect (and
		(at start (not (docked ?v)))
		(at end (undocked ?v)))
) 

(:durative-action charge
	:parameters (?v - robot)
	:duration ( = ?duration 10)
	:condition (and
		(at start (docked ?v)))
	:effect (and
		(at end (battery_charged ?v)))
)

(:durative-action charge
	:parameters (?v - robot)
	:duration ( = ?duration 10)
	:condition (and
		(at start (docked ?v)))
	:effect (and
		(at end (battery_charged ?v)))
)

)