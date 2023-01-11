(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
    turtlebot - robot
)
(:init
    (undocked turtlebot)
    (= (speed turtlebot) 1)
    (= (max_range turtlebot) 20)
    (= (state_of_charge turtlebot) 100)

)

(:goal (and
    (photographed wp3)
    (photographed wp4)
    (photographed wp5)
))
)