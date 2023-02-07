(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
    turtlebot - robot
)
(:init
    (undocked turtlebot)
    (= (speed turtlebot) 0.1)
    (= (min_charge turtlebot) 15)
    (= (state_of_charge turtlebot) 77)

)

(:goal (and
    (photographed wp0)
    (photographed wp3)
    (photographed wp5)
))
)