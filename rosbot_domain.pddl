(define (domain rosbot_domain)

    (:requirements :strips :typing :fluents )

    (:types
        waypoint 
        rosbot 
        marker 
    )

    (:predicates
        (marker_at ?m - marker ?wp - waypoint)
        (robot_at ?r - rosbot ?wp - waypoint)
        (visited ?wp - waypoint)
        (detected ?m - marker)
        (different ?wp1 - waypoint ?wp2 - waypoint)
        (not_visited ?wp - waypoint)
        (not_detected ?m - marker)
    )
    
    (:action goto
        :parameters (?r - rosbot ?wp1 - waypoint ?wp2 - waypoint)
        :precondition (and  (robot_at ?r ?wp1) (not_visited ?wp2) (different ?wp1 ?wp2))
        :effect (and (robot_at ?r ?wp2) (not (robot_at ?r ?wp1)) (visited ?wp2) (not(not_visited ?wp2)))
    )

    (:action detect
        :parameters (?r - rosbot ?wp - waypoint ?m - marker)
        :precondition (and  (robot_at ?r ?wp) (not_detected ?m)  (marker_at ?m ?wp))
        :effect (and (detected ?m) (not (not_detected ?m))
    )
)
)
