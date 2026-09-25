# Closed-loop goal: target pose and whether the path generator deems it reachable
float32 x            # target x in the odom frame [m]
float32 y            # target y in the odom frame [m]
float32 theta        # desired final heading [rad]
bool is_reachable    # false = the controller must ignore this goal
