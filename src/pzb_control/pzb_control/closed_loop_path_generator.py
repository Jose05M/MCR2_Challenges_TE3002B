import rclpy
import signal
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Bool
from pzb_interfaces.msg import Goal


# Minimum distance the robot can reach with stable control
MIN_REACHABLE_DIST = 0.05


class PathGenerator(Node):

    def __init__(self):
        super().__init__('closed_loop_path_generator')

        # Waypoints (x, y, final theta), defined in the YAML
        px = self.declare_parameter('points_x', [2.0, 2.0, 0.0, 0.0]).value
        py = self.declare_parameter('points_y', [0.0, 2.0, 2.0, 0.0]).value
        pth = self.declare_parameter('points_theta', [0.0, 0.0, 0.0, 0.0]).value
        self.min_reachable_dist = self.declare_parameter(
            'min_reachable_dist', MIN_REACHABLE_DIST).value

        self.waypoints = list(zip(px, py, pth))
        self.goal_idx = 0
        self.waiting = False   # waiting for the controller confirmation
        self.prev_x = 0.0     # the path starts at the origin
        self.prev_y = 0.0

        # Publisher and subscription
        self.pub_goal = self.create_publisher(Goal, 'goal', 10)

        self.sub_reached = self.create_subscription(
            Bool, 'goal_reached', self.reached_callback, 10)

        # Startup timer — waits 1 s so the controller is ready
        self.startup_timer = self.create_timer(1.0, self.send_first_goal)

        self.get_logger().info("Path Generator Node Started.")

    def is_reachable(self, x, y, theta):
        """
        Check whether the point is reachable.

        Criteria:
        - Distance from the previous goal > min_reachable_dist
        - theta is in [-pi, pi]
        """
        dist = np.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
        if dist < self.min_reachable_dist:
            self.get_logger().warn(f"Point ({x}, {y}) too close to previous goal, unreachable.")
            return False
        if abs(theta) > np.pi:
            self.get_logger().warn(f"Theta {theta:.2f} out of range [-pi, pi], unreachable.")
            return False
        return True

    def send_first_goal(self):
        # This timer only runs once
        self.startup_timer.cancel()
        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints completed! Mission finished.")
            return

        x, y, theta = self.waypoints[self.goal_idx]
        reachable = self.is_reachable(x, y, theta)

        msg = Goal()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        msg.is_reachable = reachable

        self.pub_goal.publish(msg)
        self.waiting = True

        if reachable:
            self.get_logger().info(
                f"Sending goal {self.goal_idx + 1}/{len(self.waypoints)}: "
                f"({x:.2f}, {y:.2f}, θ={np.degrees(theta):.1f}°)")
            self.prev_x, self.prev_y = x, y
        else:
            self.get_logger().warn(
                f"Goal {self.goal_idx + 1} marked as unreachable, skipping.")
            self.goal_idx += 1
            self.waiting = False
            self.send_next_goal()

    def reached_callback(self, msg):
        if msg.data and self.waiting:
            self.get_logger().info(f"Goal {self.goal_idx + 1} confirmed reached!")
            self.goal_idx += 1
            self.waiting = False
            self.send_next_goal()

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt! Shutting down path generator.")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    signal.signal(signal.SIGINT, node.stop_handler)
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
