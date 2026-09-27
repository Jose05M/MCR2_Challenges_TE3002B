# pzb_bringup

&ensp;&ensp;Launch files and YAML configuration for every challenge. Each launch
combines the nodes of the other `pzb_*` packages that a challenge needs; the
package has no code of its own.

## Launch files

| Launch | Challenge | Nodes | Config |
|---|---|---|---|
| `teleop.launch.py` | Week 1 | `teleop_twist_keyboard`, `rqt_plot` | — |
| `open_loop_square.launch.py` | Week 2 | `open_loop_path_generator`, `open_loop_controller` | `open_loop_square.yaml` |
| `closed_loop.launch.py part:=1\|2` | Week 3 | `odometry`, `closed_loop_path_generator`, `closed_loop_controller` | `closed_loop_part1/2.yaml` |
| `traffic_light_nav.launch.py [record:=true]` | Half-term | `odometry`, `traffic_light_detector`, `traffic_light_nav_controller`, `analysis_node` | `traffic_light_nav.yaml` |
| `line_follower.launch.py` | Week 6 | `line_detector`, `traffic_light_detector`, `line_follower_controller` | `line_follower.yaml` |
| `sign_detection.launch.py` | Week 7 | `traffic_light_detector`, `traffic_sign_detector` | `sign_detection.yaml` |
| `puzzlebot_jetson.launch.py` | Final (Jetson) | `traffic_light_detector`, `image_compressor` | `puzzlebot_final.yaml` |
| `puzzlebot_yolo.launch.py` | Final (laptop, own terminal) | `traffic_sign_detector` | `puzzlebot_final.yaml` |
| `puzzlebot_laptop.launch.py` | Final (laptop) | `line_detector`, `odometry`, `fsm_node`, `puzzlebot_controller` | `puzzlebot_final.yaml` |

&ensp;&ensp;`puzzlebot.yaml` holds the robot geometry (wheel radius and wheel base)
and is loaded by every launch that runs the odometry.

## Structure

```
pzb_bringup/
├── config/          # one YAML per challenge + puzzlebot.yaml
├── launch/          # one launch per challenge (three for the final challenge)
├── CMakeLists.txt   # installs launch/ and config/
└── package.xml
```
