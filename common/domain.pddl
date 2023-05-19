(define (domain turtlebot3)

(:requirements :strips :typing :fluents :durative-actions)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot) 
	(charge_at ?wp - waypoint)
	(photographed ?wp - waypoint)
)

(:functions 
	(distance ?wp1 ?wp2 - waypoint) 
	(speed ?v - robot)
	(min_charge ?v - robot)
	(state_of_charge ?v - robot)
	(charging_rate ?v - robot)
	(discharge_rate ?v - robot)
	(docking_duration ?v - robot)
	(traveled ?v -robot)
)


; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration (/ (distance ?from ?to) 
							  (speed ?v)))
	:condition (and 
		(at start (robot_at ?v ?from))
		(at start (>= (- (state_of_charge ?v)(* (discharge_rate ?v) (distance ?from ?to))) (min_charge ?v)))
		(over all (undocked ?v))
		)
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at end (decrease (state_of_charge ?v) (* (discharge_rate ?v) (distance ?from ?to))))
		(at end (robot_at ?v ?to))
		(at end (increase (traveled ?v) (distance ?from ?to)))
		)
)

; Docking to charger
(:durative-action dock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration (docking_duration ?v))
	:condition (and
		(at start (charge_at ?wp))
		(over all (robot_at ?v ?wp))
		(at start (undocked ?v))
		)
	:effect (and
		(at end (docked ?v))
		(at start (not (undocked ?v))))
)

; Unocking from charger
(:durative-action undock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration (docking_duration ?v))
	:condition (and
		(at start (charge_at ?wp))
		(over all (robot_at ?v ?wp))
		(at start (docked ?v)))
	:effect (and
		(at start (not (docked ?v)))
		(at end (undocked ?v)))
) 

; Charging battery
(:durative-action charge
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration (* (charging_rate ?v) (- 100 (state_of_charge ?v))))
	:condition (and
		(at start (charge_at ?wp))
		(at start (robot_at ?v ?wp))
		(over all (docked ?v))
		(at start (<= (state_of_charge ?v) 100)))
	:effect (and
		(at end (assign (state_of_charge ?v) 100))
		)
)

; Photographing an object of interest
(:durative-action inspect
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (>= (- (state_of_charge ?v) 3) (min_charge ?v))))
	:effect (and
		(at end (photographed ?wp))
		(at end (decrease (state_of_charge ?v) 3))
		)
)
)
