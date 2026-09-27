# MCR2_Challenges_TE3002B

<p align="center">
  <img alt="ROS 2 logo" src="https://avatars.githubusercontent.com/u/3979232?v=4" width="56">
  &nbsp;&nbsp;
  <img alt="Manchester Robotics logo" src="https://avatars.githubusercontent.com/u/107204750?v=4" width="56">
</p>

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white">
  <img alt="Robot" src="https://img.shields.io/badge/robot-Puzzlebot_Jetson-76B900?logo=nvidia&logoColor=white">
  <img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-vision-5C3EE8?logo=opencv&logoColor=white">
  <img alt="YOLOv8" src="https://img.shields.io/badge/YOLOv8-Ultralytics-111F68">
  <img alt="License" src="https://img.shields.io/badge/license-Apache_2.0-green">
</p>

## 1. Introduction

&ensp;&ensp;This is the ROS 2 workspace for the weekly challenges of **TE3002B -
Intelligent Robotics Implementation** (MCR2 / Tecnológico de Monterrey), a course
built around the **Puzzlebot Jetson Edition** that goes from open-loop and
closed-loop navigation to computer vision, line following, and neural-network
traffic sign detection, ending in an autonomous driving final challenge.

&ensp;&ensp;The code is organised **by function, not by challenge**: each `pzb_*`
package is one capability of the robot (vision, control, odometry, …), and each
challenge is a launch file in [pzb_bringup](src/pzb_bringup/) that combines those
packages. Later challenges reuse and extend the nodes of earlier ones, so every
stage of the project can still be run.

## 2. Packages

| Package | Summary |
|---|---|
| [pzb_interfaces](src/pzb_interfaces/) | Custom messages: `TrajectoryPose` (open loop), `Goal` (closed loop), `MotionCommand` (FSM → controller), `InferenceResult` / `Yolov8Inference` (YOLO) |
| [pzb_odometry](src/pzb_odometry/) | Dead-reckoning wheel odometry: `/VelocityEncR`, `/VelocityEncL` → `/odom` |
| [pzb_vision](src/pzb_vision/) | OpenCV: traffic light detector (HSV + blobs), scanline line detector with intersection detection, image compressor (Jetson → laptop), laptop webcam publisher |
| [pzb_detection](src/pzb_detection/) | YOLOv8 traffic sign detector and the trained model (`models/best_1.pt`) |
| [pzb_control](src/pzb_control/) | Controllers and path generators: open loop, closed loop, traffic light navigation, PD line follower, and the final challenge motion controller |
| [pzb_fsm](src/pzb_fsm/) | Final challenge state machine: decides what to do from traffic lights, signs and intersections and sends `MotionCommand`s to the controller |
| [pzb_tools](src/pzb_tools/) | Data logging to CSV (`analysis_node`), plotting script and the half-term results |
| [pzb_bringup](src/pzb_bringup/) | Launch files and YAML configuration for every challenge |

## 3. Challenges

| Challenge | Launch (`pzb_bringup`) | Config |
|---|---|---|
| Week 1 — Puzzlebot setup and teleoperation | `teleop.launch.py` | — |
| Week 2 — Open-loop control (2 m square) | `open_loop_square.launch.py` | `open_loop_square.yaml` |
| Week 3 — Closed-loop control (square / goals) | `closed_loop.launch.py part:=1\|2` | `closed_loop_part1.yaml`, `closed_loop_part2.yaml`, `puzzlebot.yaml` |
| Half-term — Waypoint navigation + traffic light | `traffic_light_nav.launch.py` | `traffic_light_nav.yaml`, `puzzlebot.yaml` |
| Week 6 — Line following + traffic light | `line_follower.launch.py` | `line_follower.yaml` |
| Week 7 — Traffic sign detection (YOLOv8) | `sign_detection.launch.py` | `sign_detection.yaml` |
| Final — Autonomous driving on the Puzzletrack | `puzzlebot_jetson.launch.py`, `puzzlebot_yolo.launch.py`, `puzzlebot_laptop.launch.py` | `puzzlebot_final.yaml`, `puzzlebot.yaml` |

## 4. Requirements

- Ubuntu 22.04 with ROS 2 Humble.
- A **Puzzlebot Jetson Edition** (Hackerboard running the MCR2 firmware). The
  micro-ROS agent runs on the Jetson, which is the one wired to the Hackerboard.
- ROS packages: `teleop_twist_keyboard`, `rqt_plot`.
- Python: `numpy`, `scipy` (odometry), `opencv-python` (vision),
  `ultralytics` (YOLOv8, `pip install ultralytics`), `pandas` + `matplotlib`
  (only for the plotting script in `pzb_tools`).

## 5. How To Build

```bash
# whole workspace
colcon build --symlink-install
source install/setup.bash

# a single package
colcon build --symlink-install --packages-select <package_name>
```

&ensp;&ensp;Build with `--symlink-install`: the YAML files in `pzb_bringup/config`
and the YOLO model are then links to `src/`, so changing a parameter (gains,
waypoints, `model_path`, …) only needs a restart of the launch, not a rebuild.
If a package was previously built without it, delete its `build/<package>` and
`install/<package>` folders once before rebuilding.

## 6. How To Run

- ### 6.1 Connect to the Puzzlebot
    &ensp;&ensp;Connect the laptop to the Jetson hotspot, open a terminal on the
    Jetson and start the micro-ROS agent (and the camera, for the vision challenges):
    ```bash
    ssh puzzlebot@10.42.0.1
    ros2 launch puzzlebot_ros micro_ros_agent.launch.py
    ```
    &ensp;&ensp;The Jetson topics (`/cmd_vel`, `/VelocityEncR`, `/VelocityEncL`,
    `/video_source/raw`) are then visible from the laptop over the ROS network.

- ### 6.2 Weekly challenges
    ```bash
    ros2 launch pzb_bringup teleop.launch.py              # Week 1
    ros2 launch pzb_bringup open_loop_square.launch.py    # Week 2
    ros2 launch pzb_bringup closed_loop.launch.py part:=1 # Week 3 (part:=2 for goals)
    ros2 launch pzb_bringup traffic_light_nav.launch.py   # Half-term (record:=true logs a CSV)
    ros2 launch pzb_bringup line_follower.launch.py       # Week 6
    ros2 launch pzb_bringup sign_detection.launch.py      # Week 7
    ```
    &ensp;&ensp;To test the vision challenges without the robot camera, publish
    the laptop webcam on `/video_source/raw` in another terminal:
    ```bash
    ros2 run pzb_vision usb_camera_publisher
    ```

- ### 6.3 Final challenge
    &ensp;&ensp;The work is split between the Jetson (camera, traffic light,
    image compression) and the laptop (line following, YOLO, odometry, FSM and
    controller). YOLO runs in its own terminal because it takes a while to load.
    ```bash
    # Jetson
    ros2 launch pzb_bringup puzzlebot_jetson.launch.py
    # Laptop, terminal 1 (YOLO traffic sign detector)
    ros2 launch pzb_bringup puzzlebot_yolo.launch.py
    # Laptop, terminal 2 (line detector, odometry, FSM, controller)
    ros2 launch pzb_bringup puzzlebot_laptop.launch.py
    ```
    &ensp;&ensp;After a STOP sign the robot stays stopped until it is resumed with:
    ```bash
    ros2 topic pub --once /fsm_command std_msgs/msg/String "{data: 'START'}"
    ```
    &ensp;&ensp;All the final challenge parameters (speeds, PD gains, turn and
    intersection distances, YOLO `model_path`, …) are in
    [puzzlebot_final.yaml](src/pzb_bringup/config/puzzlebot_final.yaml).

## 7. Team

- Josue Ureña Valencia — A01738940
- César Arellano Arellano — A00839373
- Jose Eduardo Sánchez Martínez — A01738476
- Rafael André Gamiz Salazar — A00838280

&ensp;&ensp;Developed for **TE3002B - Intelligent Robotics Implementation**, in
partnership with **Manchester Robotics (MCR2)** as industry partner, at
**Tecnológico de Monterrey**.
