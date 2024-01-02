(define (problem rosbot_problem) 
    (:domain rosbot_domain)
    (:objects
        rosbot - rosbot

        wp0 - waypoint
        wp1 - waypoint
        wp2 - waypoint
        wp3 - waypoint
        wp4 - waypoint
    
        m1 - marker
        m2 - marker
        m3 - marker
        m4 - marker
    )

(:init
    (robot_at rosbot wp0)
    (different wp0 wp1)
    (different wp1 wp0)
    (different wp0 wp2)
    (different wp2 wp0)
    (different wp0 wp3)
    (different wp3 wp0)
    (different wp0 wp4)
    (different wp4 wp0)
    (different wp1 wp2)
    (different wp2 wp1)
    (different wp1 wp3)
    (different wp3 wp1)
    (different wp1 wp4)
    (different wp4 wp1)
    (different wp2 wp3)
    (different wp3 wp2)
    (different wp2 wp4)
    (different wp4 wp2)
    (different wp3 wp4)
    (different wp4 wp3)
    (marker_at m1 wp1)
    (marker_at m2 wp2)
    (marker_at m3 wp3)
    (marker_at m4 wp4)
    (not_visited wp0)
    (not_visited wp1)
    (not_visited wp2)
    (not_visited wp3)
    (not_visited wp4)
    (not_detected m1)
    (not_detected m2)
    (not_detected m3)
    (not_detected m4)

)

(:goal (and
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (visited wp0)
    (detected m1)
    (detected m2)
    (detected m3)
    (detected m4)
))
)
