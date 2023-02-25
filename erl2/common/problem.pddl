(define (problem detectiveBot_problem)
(:domain detectiveBot_domain)
(:objects
    wp0 wp1 wp2 wp3 - waypoint
    myhome - home 
    detectiveBot - robot
)
(:init
    (start_game)
    (notSame wp0 wp1)
    (notSame wp1 wp2)
    (notSame wp2 wp3)
    (notSame wp3 wp0)

)
(:goal (and
(winningID)
)
)
)
