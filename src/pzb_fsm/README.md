# pzb_fsm

&ensp;&ensp;Decision layer of the final challenge: a finite state machine that decides
**what** the robot does from the traffic light, the traffic signs and the
intersections, and sends it to [pzb_control/puzzlebot_controller](../pzb_control/),
which decides how to move. It never publishes velocities.

```
 /traffic_light/state, /traffic_sign/state,        /motion_command
 /intersection_detected, /fsm_command ──► fsm_node ────────────────► puzzlebot_controller ──► /cmd_vel
                                             ▲                              │
                                             └──────── /motion_done ────────┘
```

## Nodes

- **fsm_node** — stores the last sign (`LEFT`, `RIGHT`, `STRAIGHT`, `STOP` as the
  pending action; `ROUND`, `GIVE_WAY`, `WORKERS` as a special behaviour) and
  sends a `MotionCommand` every cycle according to its mode:

  | Mode | Behaviour |
  |---|---|
  | `FOLLOW` | Follow the line; yellow = half speed, red = stopped |
  | `WAIT_INTERSECTION` | Stop at the zebra crossing for 1.5 s; never move on while red |
  | `TURN` | Forward 0.40 m → rotate 90° + π/13 → forward 0.10 m |
  | `STRAIGHT` | Cross the intersection (0.45 m), paused while red |
  | `SPECIAL` | `WORKERS` / `ROUND`: slower for 8 s; `GIVE_WAY`: slow, stop 2 s at the crossing, then cross |
  | `STOPPED` | `STOP` sign: follow 6 s more, then stop until `START` on `/fsm_command` |

  - In: `/traffic_light/state`, `/traffic_sign/state`, `/intersection_detected`,
    `/fsm_command`, `/motion_done`
  - Out: `/motion_command` (`MotionCommand`)

## Structure

```
pzb_fsm/
├── pzb_fsm/
│   └── fsm_node.py
├── test/
├── package.xml
└── setup.py
```
