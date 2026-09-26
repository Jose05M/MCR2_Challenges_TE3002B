#!/usr/bin/env python3
"""
Open-loop path generator.

Reads a list of waypoints (x, y, theta, time) from parameters, checks that each
one is reachable within the robot's speed limit, and sends them one at a time
on /pose. The next waypoint is sent when the controller reports /path_done.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from pzb_interfaces.msg import TrajectoryPose


class OpenLoopPathGenerator(Node):

    def __init__(self):
        super().__init__('open_loop_path_generator')

        # Path defined by the user
        self.declare_parameter('points_x', [2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('points_y', [0.0, 2.0, 2.0, 0.0])
        self.declare_parameter('points_theta', [1.57, 3.14, -1.57, 0.0])
        self.declare_parameter('points_t', [8.0, 8.0, 8.0, 8.0])

        # Limits used for the reachability check
        self.declare_parameter('v_max', 0.38)
        self.declare_parameter('w_max', 4.85)

        px = self.get_parameter('points_x').value
        py = self.get_parameter('points_y').value
        pth = self.get_parameter('points_theta').value
        pt = self.get_parameter('points_t').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value

        self.waypoints = [
            {'x': px[i], 'y': py[i], 'theta': pth[i], 't': pt[i]}
            for i in range(len(px))
        ]
        self.index = 0
        self.current_x = 0.0
        self.current_y = 0.0

        self.pub_pose = self.create_publisher(TrajectoryPose, '/pose', 10)
        self.sub_done = self.create_subscription(Bool, '/path_done', self._on_done_cb, 10)

        self.get_logger().info('Path generator ready. Checking path feasibility...')

        # One-shot timer: gives the controller time to start before the first point
        self.timer = self.create_timer(1.0, self._send_first)

    def _send_first(self):
        self.timer.cancel()
        self._publish_next()

    def _validate_reachability(self, wp):
        """Return True if the waypoint can be reached in its time at v_max."""
        dx = wp['x'] - self.current_x
        dy = wp['y'] - self.current_y
        dist = math.hypot(dx, dy)

        t_min_straight = dist / self.v_max if dist > 0 else 0.0

        if wp['t'] < t_min_straight:
            self.get_logger().error(
                f'Unreachable point. Needs at least {t_min_straight:.2f}s '
                f'to travel {dist:.2f}m at {self.v_max}m/s. Given: {wp["t"]}s'
            )
            return False

        v_req = dist / wp['t'] if wp['t'] > 0 else 0.0
        self.get_logger().info(
            f'Point {self.index + 1} reachable. Required linear speed: {v_req:.3f} m/s')
        return True

    def _on_done_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Controller reports arrival. Sending next point...')
            self._publish_next()

    def _publish_next(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('Path completed.')
            return

        wp = self.waypoints[self.index]

        if not self._validate_reachability(wp):
            self.get_logger().warn('Aborting path: physical limits exceeded.')
            return

        msg = TrajectoryPose()
        msg.x = float(wp['x'])
        msg.y = float(wp['y'])
        msg.theta = float(wp['theta'])
        msg.t_arrival = float(wp['t'])

        self.pub_pose.publish(msg)
        self.get_logger().info(
            f'Sending point {self.index + 1} (X: {msg.x}, Y: {msg.y}) to the controller.')

        # The next segment starts where this one ends
        self.current_x = wp['x']
        self.current_y = wp['y']
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OpenLoopPathGenerator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
