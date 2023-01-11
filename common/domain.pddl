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
	(charge_at ?wp - waypoint) ; Charger waypoint
	(battery_charged ?v - robot)
	(photographed ?wp - waypoint)
)

(:functions
	(distance ?wp1 ?wp2 - waypoint) 
	(speed ?v - robot)
	(max_range ?v - robot)
    (state_of_charge ?v - robot)
)


;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration (/ (distance ?from ?to) 
							  (speed ?v)))
	:condition (and 
		(at start (robot_at ?v ?from))
		(at start (>= (state_of_charge ?v)
                	(* 100 (/ (distance ?from ?to) (max_range ?v)))))
		(over all (undocked ?v))
		)
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at start (decrease (state_of_charge ?v)
                            (* 100 (/ (distance ?from ?to) (max_range ?v)))))
		(at end (visited ?to))
		(at end (robot_at ?v ?to)))
)

(:durative-action dock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 2)
	:condition (and
		(at start (charge_at ?wp))
		(over all (robot_at ?v ?wp))
		(at start (undocked ?v)))
	:effect (and
		(at end (docked ?v))
		(at start (not (undocked ?v))))
)

(:durative-action undock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 2)
	:condition (and
		(over all (charge_at ?wp))
		(at start (docked ?v)))
	:effect (and
		(at start (not (docked ?v)))
		(at end (undocked ?v)))
) 

(:durative-action charge
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (charge_at ?wp))
		(over all (docked ?v))
		(over all (robot_at ?v ?wp))
		(at start (<= (state_of_charge ?v) 100)))
	:effect (and
		(at end (assign (state_of_charge ?v) 100))
		)
)

(:durative-action inspect
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 20)
	:condition (and
		(over all (robot_at ?v ?wp)))
	:effect (and
		(at end (photographed ?wp))
		(at end (decrease (state_of_charge ?v) 3))
		)
)
)