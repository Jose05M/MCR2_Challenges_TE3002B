# pzb_control

&ensp;&ensp;Controllers and path generators. Each challenge adds a controller that
builds on the previous ones, and all of them publish `/cmd_vel`
(`geometry_msgs/Twist`).

## Nodes

- **open_loop_path_generator** (Week 2) — sends the waypoints `(x, y, theta, t)`
  one at a time after checking they can be reached in their time.
  - Out: `/pose` (`TrajectoryPose`) — In: `/path_done`
- **open_loop_controller** (Week 2) — reaches each waypoint with a timed
  turn + straight FSM, without feedback (slip / inertia correction factors).
  - In: `/pose` — Out: `/cmd_vel`, `/path_done`
- **closed_loop_path_generator** (Week 3) — sends the goals `(x, y, theta)` one at
  a time, skipping the unreachable ones.
  - Out: `goal` (`Goal`) — In: `goal_reached`
- **closed_loop_controller** (Week 3) — with `/odom` feedback: rotate towards the
  goal, advance with a proportional speed and a ramp, and optionally rotate to
  the final heading.
  - In: `goal`, `odom` — Out: `cmd_vel`, `goal_reached`
- **traffic_light_nav_controller** (half-term) — proportional waypoint navigation
  whose speed is scaled by the traffic light (green × 1, yellow × 0.4, red × 0).
  - In: `/odom`, `/traffic_light/state` — Out: `/cmd_vel`
- **line_follower_controller** (Week 6) — PD on the line error with a different
  gain on straights and curves and a speed ramp; stops on red, halves the speed on
  yellow.
  - In: `/line_error`, `/traffic_light/state` — Out: `/cmd_vel`
- **puzzlebot_controller** (final challenge) — executes the commands of
  [pzb_fsm](../pzb_fsm/): `FOLLOW_LINE` (the line-following PD), `STOP`, `DRIVE`
  and `ROTATE` (measured with odometry, reported on `/motion_done`).
  - In: `/motion_command` (`MotionCommand`), `/line_error`, `/odom` — Out:
    `/cmd_vel`, `/motion_done`

## Structure

```
pzb_control/
├── pzb_control/
│   ├── open_loop_path_generator.py      # Week 2
│   ├── open_loop_controller.py          # Week 2
│   ├── closed_loop_path_generator.py    # Week 3
│   ├── closed_loop_controller.py        # Week 3
│   ├── traffic_light_nav_controller.py  # half-term
│   ├── line_follower_controller.py      # Week 6
│   └── puzzlebot_controller.py          # final challenge
├── test/
├── package.xml
└── setup.py
```
