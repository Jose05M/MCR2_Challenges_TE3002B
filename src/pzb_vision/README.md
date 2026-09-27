# pzb_vision

&ensp;&ensp;Computer vision with OpenCV: traffic light detection, line detection
with intersection (zebra crossing) detection, and the camera utilities that move
images between the Jetson and the laptop.

## Nodes

- **traffic_light_detector** — HSV segmentation + blob filters (area,
  circularity, convexity, colour dominance) with a `SimpleBlobDetector` fallback.
  A red lock keeps `RED` until green is seen.
  - In: `/video_source/raw` (`sensor_msgs/Image`)
  - Out: `/traffic_light/state` (`std_msgs/String`: `RED`, `YELLOW`, `GREEN`, `UNKNOWN`)
- **line_detector** — Otsu threshold + 5 weighted scanlines near the bottom of
  the image; publishes the offset of the black line and flags the zebra crossing
  when the bottom band has almost no line. Reads the raw image (`compressed:
  false`) or the JPEG from `image_compressor` (final challenge).
  - In: `/video_source/raw` or `/video_source/yolo/compressed`
  - Out: `/line_error` (`std_msgs/Float32`, px, positive = line on the left),
    `/intersection_detected` (`std_msgs/Bool`)
- **image_compressor** — runs on the Jetson: resizes the camera image to 416×416,
  converts it to gray and sends it as JPEG (~5 KB instead of ~900 KB per frame).
  - In: `/video_source/raw` — Out: `/video_source/yolo/compressed` (`sensor_msgs/CompressedImage`)
- **usb_camera_publisher** — test utility: publishes the laptop webcam on
  `/video_source/raw`.

## Structure

```
pzb_vision/
├── pzb_vision/
│   ├── traffic_light_detector.py
│   ├── line_detector.py
│   ├── image_compressor.py
│   └── usb_camera_publisher.py
├── test/
├── package.xml
└── setup.py
```
