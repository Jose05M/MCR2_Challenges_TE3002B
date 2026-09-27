# pzb_interfaces

&ensp;&ensp;Custom ROS 2 messages shared by all the Puzzlebot packages.

## Messages

| Message | Fields | Used between |
|---|---|---|
| `TrajectoryPose` | `x`, `y`, `theta`, `t_arrival` | `open_loop_path_generator` → `open_loop_controller` |
| `Goal` | `x`, `y`, `theta`, `is_reachable` | `closed_loop_path_generator` → `closed_loop_controller` |
| `MotionCommand` | `mode` (`FOLLOW_LINE`, `STOP`, `DRIVE`, `ROTATE`), `linear_scale`, `angular_scale`, `distance`, `speed`, `angle` | `fsm_node` → `puzzlebot_controller` |
| `InferenceResult` / `Yolov8Inference` | class name + bounding box / list of detections | YOLO detections (course template, kept for bounding boxes) |

## Structure

```
pzb_interfaces/
├── msg/
│   ├── TrajectoryPose.msg
│   ├── Goal.msg
│   ├── MotionCommand.msg
│   ├── InferenceResult.msg
│   └── Yolov8Inference.msg
├── CMakeLists.txt
└── package.xml
```
