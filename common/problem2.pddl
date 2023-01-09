(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp3 wp4 wp5 - waypoint
    turtlebot - robot
)
(:init
    (robot_at turtlebot wp0)


    (undocked turtlebot)



    (battery_charged turtlebot)

    (= (distance wp0 wp3) 30)
    (= (distance wp0 wp4) 12)
    (= (distance wp0 wp5) 20)
    (= (distance wp3 wp0) 10)
    (= (distance wp4 wp0) 12)
    (= (distance wp5 wp0) 20)
    (= (distance wp3 wp4) 8)
    (= (distance wp3 wp5) 9)
    (= (distance wp5 wp3) 9)
    (= (distance wp4 wp5) 21)
    (= (distance wp5 wp4) 21)

    (= (speed turtlebot) 1)

)
(:goal (and
    (visited wp3)
    (visited wp4)
    (visited wp5)
))
)
