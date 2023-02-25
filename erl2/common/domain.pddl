(define (domain detectiveBot_domain)

(:requirements :strips :typing :disjunctive-preconditions :durative-actions :negative-preconditions)

(:types
	home 
	waypoint
	robot
)

(:predicates
	(robot_wp ?wp - waypoint)
	(robot_home ?h - home)
	(hint_taken ?wp - waypoint)
	(ready_to_grip)
    	(check_hypo ?wp - waypoint)
	(notSame ?wp1 ?wp2 - waypoint)
    	(start_game)
    	(winningID)
)

(:durative-action initialization
	:parameters (?h - home)
	:duration ( = ?duration 60)
	:condition (and
        (at start (start_game)))
	:effect (and
	(at end  (robot_home ?h))
		(at start (not (start_game))))
)


;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_wp ?from))
		(at start (notSame ?from ?to)))
		;(at start (hint_taken ?from)))
		;(at start  (check_hypo ?from)))
	:effect (and
		;(at end (visited ?to))
		(at end (robot_wp ?to))
		(at end (ready_to_grip))
		(at start (not (robot_wp ?from))))
)



;; Localise
(:durative-action move_arm
	:parameters (?from - waypoint  )
	:duration ( = ?duration 6)
	:condition (and
		(at start (ready_to_grip))
    	(at start (robot_wp ?from)))
	:effect (and 
		(at end (hint_taken ?from))
		(at start (not (ready_to_grip))))
		
)

;; Dock to charge
(:durative-action check_hypothesis
	:parameters (?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (hint_taken ?wp))
		(at start (robot_wp ?wp )))
	:effect (and
		(at end (check_hypo ?wp))
		(at start (not (hint_taken ?wp))))
)

(:durative-action go_home
	:parameters (?from - waypoint ?to - home)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_wp ?from)))
		;(at start (check_hypo ?from)))
	:effect (and
		(at end (robot_home ?to))
		(at start (not (robot_wp ?from))))
)

(:durative-action leave_home
	:parameters (?from - home ?to - waypoint )
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_home ?from)))
	:effect (and
        (at start (not (robot_home ?from)))
		(at end (robot_wp ?to))
		(at end (ready_to_grip)))
		;(at end (visited ?to)))
)

(:durative-action oracle
	:parameters (?h - home ?wp - waypoint) 
	:duration ( = ?duration 7)
	:condition (and
		(at start (robot_home ?h))
		(at start (check_hypo ?wp)))
	:effect (and
        	(at end (winningID))
		(at start (not (check_hypo ?wp))))
)
)
