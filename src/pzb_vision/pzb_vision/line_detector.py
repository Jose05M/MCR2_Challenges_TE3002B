#!/usr/bin/env python3
"""
Line detector (scanlines).

Thresholds the camera image (Otsu), scans 5 weighted rows near the bottom and
publishes on /line_error the pixel offset between the image centre and the
centre of the black line. When the bottom band has almost no line (zebra
crossing) it publishes True on /intersection_detected.

Input: raw Image (compressed: false) or the JPEG CompressedImage produced by
image_compressor (compressed: true).
"""
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class LineDetector(Node):

    def __init__(self):

        super().__init__('line_detector')

        # PARAMETERS
        self.declare_parameter('camera_topic', '/video_source/yolo/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('line_error_topic', '/line_error')
        self.declare_parameter('intersection_topic', '/intersection_detected')
        self.declare_parameter('debug_view', True)
        self.declare_parameter('crop_percent', 0.20)

        camera_topic = self.get_parameter('camera_topic').value
        self.compressed = self.get_parameter('compressed').value
        error_topic = self.get_parameter('line_error_topic').value
        intersection_topic = self.get_parameter('intersection_topic').value
        self.debug_view = self.get_parameter('debug_view').value
        self.crop_percent = self.get_parameter('crop_percent').value

        # ROS
        if self.compressed:
            self.sub = self.create_subscription(
                CompressedImage, camera_topic, self.image_callback, qos)
        else:
            self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.pub_error = self.create_publisher(Float32, error_topic, 10)
        self.pub_intersection = self.create_publisher(Bool, intersection_topic, 10)

        # INTERNAL STATE
        self.last_error = 0.0
        self.get_logger().info('Scanline Line Follower Started')

    # IMAGE CALLBACK
    def image_callback(self, msg):
        if self.compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        frame = cv2.resize(frame, (640, 480))

        height, width = frame.shape[:2]

        # ROI
        crop_x = int(width * self.crop_percent)

        roi = frame[
            :,
            crop_x:width - crop_x
        ]
        roi_height, roi_width = roi.shape[:2]

        # PREPROCESSING
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # MORPHOLOGY
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 5))
        binary = cv2.erode(binary, kernel, iterations=3)
        binary = cv2.dilate(binary, kernel, iterations=3)

        # ROI TO DETECT THE INTERSECTION
        lost_roi = binary[int(roi_height * 0.88):, :]
        white_pixels = cv2.countNonZero(lost_roi)
        total_pixels = (lost_roi.shape[0] * lost_roi.shape[1])
        white_ratio = white_pixels / total_pixels
        line_lost = white_ratio < 0.1

        # SCANLINES
        scan_rows = [int(roi_height * 0.80), int(roi_height * 0.84),
                     int(roi_height * 0.88), int(roi_height * 0.92), int(roi_height * 0.96)]
        weights = [1, 2, 3, 4, 5]

        center_image = roi_width // 2
        errors = []
        valid_weights = []
        output = roi.copy()

        # PROCESS EACH ROW
        for i, y in enumerate(scan_rows):

            # Skip rows out of range
            if y >= roi_height:
                continue

            row_pixels = binary[y]
            white_pixels = np.where(row_pixels == 255)[0]

            if len(white_pixels) > 0:

                # split into groups of consecutive pixels
                segments = np.split(white_pixels, np.where(np.diff(white_pixels) > 10)[0] + 1)

                best_segment = None
                largest_size = 0

                for segment in segments:
                    segment_size = len(segment)

                    # ignore small noise
                    if segment_size < 20:
                        continue

                    segment_width = (segment[-1] - segment[0])
                    # ignore huge blobs
                    if segment_width > (roi_width * 0.7):
                        continue

                    # keep the largest one
                    if segment_size > largest_size:
                        largest_size = segment_size
                        best_segment = segment

                # no valid segment in this row: skip it
                if best_segment is None:
                    continue

                left = best_segment[0]
                right = best_segment[-1]

                center_line = (left + right) // 2

                error = (center_image - center_line)
                errors.append(error)

                valid_weights.append(weights[i])

                # DEBUG
                if self.debug_view:

                    # horizontal line
                    cv2.line(output, (0, y), (roi_width, y), (0, 255, 255), 1)

                    # left edge
                    cv2.circle(output, (left, y), 4, (0, 255, 0), -1)

                    # right edge
                    cv2.circle(output, (right, y), 4, (255, 0, 0), -1)

                    # line centre
                    cv2.circle(output, (center_line, y), 4, (0, 0, 255), -1)

        # COMPUTE FINAL ERROR
        if line_lost:
            final_error = 0.0

        else:
            final_error = self.last_error

            if len(errors) > 0:
                final_error = np.average(errors, weights=valid_weights)
                self.last_error = final_error

        # PUBLISH ERROR
        error_msg = Float32()
        error_msg.data = float(final_error)
        self.pub_error.publish(error_msg)

        intersection_msg = Bool()
        intersection_msg.data = line_lost
        self.pub_intersection.publish(intersection_msg)

        # DEBUG
        if self.debug_view:
            # image centre
            cv2.line(output, (center_image, 0), (center_image, roi_height), (255, 255, 255), 2)
            cv2.putText(output, f'Error: {final_error:.1f}', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # draw the intersection ROI
            cv2.rectangle(output, (0, int(roi_height * 0.88)), (roi_width, roi_height),
                          (0, 255, 255), 2)

            # show ratio
            cv2.putText(output, f'LINE: {white_ratio:.2f}', (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # show detection
            if line_lost:
                cv2.putText(output, 'LINE LOST', (20, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

            cv2.imshow("ROI", output)
            cv2.imshow("Binary", binary)
            cv2.imshow("real", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
