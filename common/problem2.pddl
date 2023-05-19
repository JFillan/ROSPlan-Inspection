(define (problem task)
(:domain turtlebot3)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 - waypoint
    turtlebot - robot
)
(:init
    (robot_at turtlebot wp0)

    (undocked turtlebot)


    (charge_at wp0)
    (charge_at wp1)


    (= (distance wp0 wp0) 100)
    (= (distance wp1 wp0) 5.59464)
    (= (distance wp0 wp1) 5.59464)
    (= (distance wp1 wp1) 100)
    (= (distance wp2 wp0) 2.94109)
    (= (distance wp0 wp2) 2.94109)
    (= (distance wp2 wp1) 5.80086)
    (= (distance wp1 wp2) 5.80086)
    (= (distance wp2 wp2) 100)
    (= (distance wp3 wp0) 7.15122)
    (= (distance wp0 wp3) 7.15122)
    (= (distance wp3 wp1) 4.66476)
    (= (distance wp1 wp3) 4.66476)
    (= (distance wp3 wp2) 5.17397)
    (= (distance wp2 wp3) 5.17397)
    (= (distance wp3 wp3) 100)
    (= (distance wp4 wp0) 7.36614)
    (= (distance wp0 wp4) 7.36614)
    (= (distance wp4 wp1) 2.28035)
    (= (distance wp1 wp4) 2.28035)
    (= (distance wp4 wp2) 6.73573)
    (= (distance wp2 wp4) 6.73573)
    (= (distance wp4 wp3) 3.49857)
    (= (distance wp3 wp4) 3.49857)
    (= (distance wp4 wp4) 100)
    (= (distance wp5 wp0) 7.01783)
    (= (distance wp0 wp5) 7.01783)
    (= (distance wp5 wp1) 3.82753)
    (= (distance wp1 wp5) 3.82753)
    (= (distance wp5 wp2) 8.56154)
    (= (distance wp2 wp5) 8.56154)
    (= (distance wp5 wp3) 8.49058)
    (= (distance wp3 wp5) 8.49058)
    (= (distance wp5 wp4) 5.60446)
    (= (distance wp4 wp5) 5.60446)
    (= (distance wp5 wp5) 100)
    (= (distance wp6 wp0) 3.25576)
    (= (distance wp0 wp6) 3.25576)
    (= (distance wp6 wp1) 5.50091)
    (= (distance wp1 wp6) 5.50091)
    (= (distance wp6 wp2) 5.99083)
    (= (distance wp2 wp6) 5.99083)
    (= (distance wp6 wp3) 8.90056)
    (= (distance wp3 wp6) 8.90056)
    (= (distance wp6 wp4) 7.74726)
    (= (distance wp4 wp6) 7.74726)
    (= (distance wp6 wp5) 4.90408)
    (= (distance wp5 wp6) 4.90408)
    (= (distance wp6 wp6) 100)
    (= (distance wp7 wp0) 2.54951)
    (= (distance wp0 wp7) 2.54951)
    (= (distance wp7 wp1) 8.14125)
    (= (distance wp1 wp7) 8.14125)
    (= (distance wp7 wp2) 4.14367)
    (= (distance wp2 wp7) 4.14367)
    (= (distance wp7 wp3) 9.1586)
    (= (distance wp3 wp7) 9.1586)
    (= (distance wp7 wp4) 9.84073)
    (= (distance wp4 wp7) 9.84073)
    (= (distance wp7 wp5) 9.31933)
    (= (distance wp5 wp7) 9.31933)
    (= (distance wp7 wp6) 4.82701)
    (= (distance wp6 wp7) 4.82701)
    (= (distance wp7 wp7) 100)

    (= (speed turtlebot) 0.1)

    (= (min_charge turtlebot) 15)

    (= (state_of_charge turtlebot) 100)

    (= (charging_rate turtlebot) 0.5)

    (= (discharge_rate turtlebot) 4)

    (= (docking_duration turtlebot) 3)

    (= (traveled turtlebot) 0)

)
(:goal (and
    (photographed wp2)
    (photographed wp3)
    (photographed wp4)
    (photographed wp5)
    (photographed wp6)
    (photographed wp7)
))
(:metric minimize (traveled turtlebot))
)
