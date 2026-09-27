# pzb_detection

&ensp;&ensp;Neural-network detection: the YOLOv8 traffic sign detector and the model
trained by the team.

## Nodes

- **traffic_sign_detector** — runs YOLOv8 at 5 Hz, keeps the closest sign whose
  box area is in range, double-checks `LEFT` / `RIGHT` with the arrow shape and
  publishes a label after 3 consecutive detections, only when it changes. Reads
  the raw image (`compressed: false`, Week 7) or the JPEG from `image_compressor`
  (final challenge). Classes: `GIVE_WAY`, `LEFT`, `RIGHT`, `ROUND`, `STOP`,
  `STRAIGHT`, `WORKERS`.
  - In: `/video_source/raw` or `/video_source/yolo/compressed`
  - Out: `/traffic_signals/state` (Week 7) or `/traffic_sign/state` (final)
    (`std_msgs/String`)

## Model

&ensp;&ensp;[models/best_1.pt](models/best_1.pt) is the trained model, used in both
Week 7 and the final challenge. Another model can be used by setting its full
path in `model_path` in the YAML of the launch file.

## Structure

```
pzb_detection/
├── models/
│   └── best_1.pt
├── pzb_detection/
│   └── traffic_sign_detector.py
├── test/
├── package.xml
└── setup.py
```
