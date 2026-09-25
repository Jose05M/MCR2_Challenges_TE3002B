#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32

from rclpy.qos import qos_profile_sensor_data

import time


class AnalysisNode(Node):

    def __init__(self):

        super().__init__('analysis_node')

        # =====================================
        # TIME
        # =====================================

        self.start_time = time.time()

        # =====================================
        # LAST RECEIVED DATA
        # =====================================

        self.last_v = 0.0
        self.last_w = 0.0

        self.last_traffic = "NONE"

        self.last_enc_r = 0.0
        self.last_enc_l = 0.0

        self.last_laser = 0.0

        # =====================================
        # CSV FILE
        # =====================================

        self.csv_file = open('/home/ed/ros2_challenge_mcr2/ros2_challenge2/src/mcr2_challenge/media/robot_analysis.csv', 'w')

        self.csv_file.write(
            'time,x,y,v,w,traffic,enc_r,enc_l,laser\n'
        )

        # =====================================
        # SUBSCRIBERS
        # =====================================

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            '/traffic_light/state',
            self.traffic_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32,
            '/VelocityEncR',
            self.enc_r_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32,
            '/VelocityEncL',
            self.enc_l_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32,
            '/LaserDistance',
            self.laser_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Analysis node running...')

    # =====================================
    # TIME
    # =====================================

    def current_time(self):

        return time.time() - self.start_time

    # =====================================
    # CALLBACKS
    # =====================================

    def odom_callback(self, msg):

        t = self.current_time()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        row = (
            f'{t},'
            f'{x},'
            f'{y},'
            f'{self.last_v},'
            f'{self.last_w},'
            f'{self.last_traffic},'
            f'{self.last_enc_r},'
            f'{self.last_enc_l},'
            f'{self.last_laser}\n'
        )

        self.csv_file.write(row)

    def cmd_callback(self, msg):

        self.last_v = msg.linear.x
        self.last_w = msg.angular.z

    def traffic_callback(self, msg):

        self.last_traffic = msg.data

    def enc_r_callback(self, msg):

        self.last_enc_r = msg.data

    def enc_l_callback(self, msg):

        self.last_enc_l = msg.data

    def laser_callback(self, msg):

        self.last_laser = msg.data


# =====================================
# MAIN
# =====================================

def main(args=None):

    rclpy.init(args=args)

    node = AnalysisNode()

    try:

        while rclpy.ok():

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:

        print('\nStopping analysis node...')

    finally:

        node.csv_file.close()

        print('\nCSV file saved successfully.')

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
