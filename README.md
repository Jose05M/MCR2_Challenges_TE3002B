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

> **Status:** this repository is currently being refactored. Code structure and
> per-package documentation will change as each challenge is cleaned up.

## 2. Packages

| Package | Summary |
|---|---|
| [trajectory_controller](src/trajectory_controller/) | Open-loop path generator + time-based FSM controller (turn / straight) with reachability check |
| [trajectory_msgs_custom](src/trajectory_msgs_custom/) | Custom `TrajectoryPose` message (`x`, `y`, `theta`, `t_arrival`) |
| [puzzlebot_control](src/puzzlebot_control/) | Closed-loop point-to-point controller driven by wheel odometry, square path and goal generator |
| [puzzlebot_msgs](src/puzzlebot_msgs/) | Custom `Goal` message (`x`, `y`, `theta`, `is_reachable`) |
| [mcr2_challenge](src/mcr2_challenge/) | Waypoint navigation that obeys a traffic light detected with HSV + blob detection; includes a data-logging node and plots |
| [puzzlebot_line_follower](src/puzzlebot_line_follower/) | Camera-based line detection + PD line-following controller combined with the traffic light detector |
| [yolov8_ros2](src/yolov8_ros2/) | YOLOv8 traffic sign recognition node (based on the course template) |
| [yolo_msg](src/yolo_msg/) | `InferenceResult` / `Yolov8Inference` messages for YOLO detections |
| [mcr2_puzzlebot](src/mcr2_puzzlebot/) | Full autonomous driving stack split between Jetson and laptop: lane following, traffic lights, YOLOv8 traffic signs, intersections, and an FSM |

## 3. Requirements

- Ubuntu 22.04 with ROS 2 Humble.
- A **Puzzlebot Jetson Edition** (Hackerboard running the MCR2 firmware) and the
  `micro_ros_agent` to bridge it to ROS 2.
- Python: `numpy`, `scipy` (odometry), `opencv-python` + `cv_bridge` (vision
  packages), `ultralytics` (YOLOv8 packages)

## 4. How To Build

```bash
# whole workspace
colcon build
source install/setup.bash

# a single package
colcon build --packages-select <package_name>
```

## 5. Team

- Josue Ureña Valencia — A01738940
- César Arellano Arellano — A00839373
- Jose Eduardo Sánchez Martínez — A01738476
- Rafael André Gamiz Salazar — A00838280

&ensp;&ensp;Developed for **TE3002B - Intelligent Robotics Implementation**, in
partnership with **Manchester Robotics (MCR2)** as industry partner, at
**Tecnológico de Monterrey**.
