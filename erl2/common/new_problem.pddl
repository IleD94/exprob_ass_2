(define (problem task)
(:domain detectivebot_domain)
(:objects
    myhome - home
    wp0 wp1 wp2 wp3 - waypoint
    detectivebot - robot
)
(:init
    (robot_wp wp1)





    (notsame wp0 wp1)
    (notsame wp1 wp2)
    (notsame wp2 wp3)
    (notsame wp3 wp0)



)
(:goal (and
    (winningid)
))
)
