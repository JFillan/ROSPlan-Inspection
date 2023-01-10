(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
    turtlebot - robot
)
(:init
    (robot_at turtlebot wp0)




    (dock_at wp2)

    (not (battery_charged turtlebot))

    (= (distance wp0 wp0) 0)
    (= (distance wp1 wp0) 4.48219)
    (= (distance wp0 wp1) 4.48219)
    (= (distance wp1 wp1) 0)
    (= (distance wp2 wp2) 0)
    (= (distance wp3 wp0) 2.23607)
    (= (distance wp0 wp3) 2.23607)
    (= (distance wp3 wp1) 5.41202)
    (= (distance wp1 wp3) 5.41202)
    (= (distance wp3 wp3) 0)
    (= (distance wp4 wp0) 6.72012)
    (= (distance wp0 wp4) 6.72012)
    (= (distance wp4 wp2) 4.18688)
    (= (distance wp2 wp4) 4.18688)
    (= (distance wp4 wp4) 0)
    (= (distance wp5 wp0) 7.33757)
    (= (distance wp0 wp5) 7.33757)
    (= (distance wp5 wp5) 0)
    (= (distance wp6 wp0) 2.54951)
    (= (distance wp0 wp6) 2.54951)
    (= (distance wp6 wp1) 6.84763)
    (= (distance wp1 wp6) 6.84763)
    (= (distance wp6 wp2) 6.70298)
    (= (distance wp2 wp6) 6.70298)
    (= (distance wp6 wp3) 3.80789)
    (= (distance wp3 wp6) 3.80789)
    (= (distance wp6 wp4) 4.54533)
    (= (distance wp4 wp6) 4.54533)
    (= (distance wp6 wp5) 5.24786)
    (= (distance wp5 wp6) 5.24786)
    (= (distance wp6 wp6) 0)

    (= (speed turtlebot) 1)

    (= (max_range turtlebot) 20)

    (= (state_of_charge turtlebot) 100)

)
(:goal (and
    (visited wp3)
    (visited wp4)
    (visited wp5)
))
)
