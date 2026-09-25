# Open-loop waypoint: target position plus the time allowed to reach it
float64 x          # target x in the world frame [m]
float64 y          # target y in the world frame [m]
float64 theta      # desired final heading [rad] (0 = no constraint)
float64 t_arrival  # time to complete this segment [s]
