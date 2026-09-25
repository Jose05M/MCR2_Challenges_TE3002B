import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np

# CORRECCIÓN LEFT / RIGHT
def detect_arrow_direction(roi):

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    _, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return "UNKNOWN"

    cnt = max(contours, key=cv2.contourArea)
    M = cv2.moments(cnt)

    if M["m00"] == 0:
        return "UNKNOWN"

    cx = int(M["m10"] / M["m00"])

    leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
    rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])

    dist_left = abs(cx - leftmost[0])
    dist_right = abs(rightmost[0] - cx)

    if dist_left > dist_right:
        return "LEFT"
    else:
        return "RIGHT"


class YoloInference(Node):

#best-> normal, anterior. 
#best1-> gray,ruido
#best2 _gray, sin ruido
#best3 -> color
    def __init__(self):
        super().__init__('yolov8_recognition')
        self.model = YOLO("/home/ed/Downloads/yolo_train/best_1.pt")

        self.img = None
        self.valid_img = False
        self.last_state = ""

        self.sub = self.create_subscription(Image,'/video_source/raw',self.camera_callback,10)
        self.state_pub = self.create_publisher(String,'/traffic_signals/state',10)

        self.timer = self.create_timer(1/20,self.timer_callback)

    def camera_callback(self, msg):
        try:
            frame = np.frombuffer(msg.data,dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.img = frame.copy()
            self.valid_img = True

        except Exception as e:
            self.get_logger().info(f'Image conversion failed: {e}')

    def timer_callback(self):
        if not self.valid_img:
            return

        # RESIZE 416x416
        frame = cv2.resize(self.img, (416,416))

        # GRAYSCALE
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # YOLO necesita 3 canales
        gray_3ch = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)

        # INFERENCIA
        results = self.model(gray_3ch,conf=0.4,imgsz=416,verbose=False)

        best_label = None
        best_area = 0

        # DETECCIONES
        for r in results:
            for box in r.boxes:

                x1, y1, x2, y2 = map(int,box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                label = self.model.names[cls]

                # ÁREA
                w = x2 - x1
                h = y2 - y1
                area = w * h
                MIN_AREA = 30000
                MAX_AREA = 80000

                if area < MIN_AREA:
                    continue

                if area > MAX_AREA:
                    continue

                self.get_logger().info(f"{label} area = {area}")

                # ROI
                roi = frame[y1:y2, x1:x2]

                if roi.size == 0:
                    continue

                # CORRECCIÓN LEFT / RIGHT
                if label == "LEFT" or label == "RIGHT":
                    corrected = detect_arrow_direction(roi)

                    if corrected != "UNKNOWN":
                        label = corrected

                # DETECCIÓN MÁS CERCANA
                if area > best_area:
                    best_area = area
                    best_label = label

        # FRAME ANOTADO
        annotated_frame = results[0].plot()

        # TEXTO FRONT
        if best_label is not None:

            cv2.putText(
                annotated_frame,
                f"FRONT: {best_label}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,0),
                3
            )

        cv2.imshow(
            "YOLO Detection",
            annotated_frame
        )

        cv2.waitKey(1)

        if best_label is not None and best_label != self.last_state:
            state_msg = String()
            state_msg.data = best_label
            self.state_pub.publish(state_msg)
            self.last_state = best_label
            self.get_logger().info(f"Published state: {best_label}")


def main(args=None):

    rclpy.init(args=args)
    node = YoloInference()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()