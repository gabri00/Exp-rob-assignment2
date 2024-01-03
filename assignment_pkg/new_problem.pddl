(define (problem task)
(:domain rosbot_domain)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    rosbot - rosbot
    m1 m2 m3 m4 - marker
)
(:init
    (marker_at m1 wp1)
    (marker_at m2 wp2)
    (marker_at m3 wp3)
    (marker_at m4 wp4)

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

    (not_visited wp0)
    (not_visited wp1)
    (not_visited wp2)
    (not_visited wp3)
    (not_visited wp4)

    (not_detected m1)
    (not_detected m2)
    (not_detected m3)
    (not_detected m4)

    (connected wp0 wp1)
    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)
    (connected wp4 wp0)

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
