(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 - waypoint
    turtlebot - robot
)
(:init
    (undocked turtlebot)
    (= (speed turtlebot) 0.1)
    (= (min_charge turtlebot) 15)
    (= (state_of_charge turtlebot) 100)

)

(:goal (and
    (photographed wp3)
    (photographed wp4)
    (photographed wp5)
    (photographed wp6)
))
;(:metric minimize (traveled))
)