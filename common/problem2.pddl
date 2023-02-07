(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
    turtlebot - robot
)
(:init
    (robot_at turtlebot wp0)


    (undocked turtlebot)


    (charge_at wp1)
    (charge_at wp2)




    (= (distance wp0 wp0) 0)
    (= (distance wp1 wp0) 4.48219)
    (= (distance wp0 wp1) 4.48219)
    (= (distance wp1 wp1) 0)
    (= (distance wp2 wp0) 7.6896)
    (= (distance wp0 wp2) 7.6896)
    (= (distance wp2 wp2) 0)
    (= (distance wp3 wp0) 2.23607)
    (= (distance wp0 wp3) 2.23607)
    (= (distance wp3 wp1) 5.41202)
    (= (distance wp1 wp3) 5.41202)
    (= (distance wp3 wp2) 6.38201)
    (= (distance wp2 wp3) 6.38201)
    (= (distance wp3 wp3) 0)
    (= (distance wp4 wp0) 6.72012)
    (= (distance wp0 wp4) 6.72012)
    (= (distance wp4 wp2) 4.18688)
    (= (distance wp2 wp4) 4.18688)
    (= (distance wp4 wp3) 6.70522)
    (= (distance wp3 wp4) 6.70522)
    (= (distance wp4 wp4) 0)
    (= (distance wp5 wp0) 7.33757)
    (= (distance wp0 wp5) 7.33757)
    (= (distance wp5 wp4) 6.40312)
    (= (distance wp4 wp5) 6.40312)
    (= (distance wp5 wp5) 0)

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
