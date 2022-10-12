(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
    turtlebot - robot
)
(:init
    (robot_at turtlebot wp0)
)

(:goal (and
    (visited wp2)
    (visited wp3)
    (visited wp1)
    (visited wp4)
))
)