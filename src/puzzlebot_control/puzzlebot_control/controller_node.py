import rclpy
import numpy as np
import signal

from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from puzzlebot_msgs.msg import Goal


def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
        result += 2 * np.pi
    return result - np.pi


class ClosedLoopController(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Ganancias
        self.Kv = self.declare_parameter('Kv', 0.5).value
        self.Kw = self.declare_parameter('Kw', 0.5).value

        # Rampa lineal
        self.V_actual = 0.0
        self.max_accel = 0.3

        # Límites de velocidad
        self.V_min = 0.07
        self.V_max = 0.40
        self.W_fixed = 1.0

        # Umbrales
        self.align_threshold = 0.05
        self.goal_threshold  = 0.10
        self.goal_reached_d  = 0.04

        # Estado
        self.phase          = 'wait'   # wait | rotate | advance
        self.finished       = False
        self.returning_home = False
        self.current_goal   = None     # Goal msg activo

        # Pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # dt
        self.first     = True
        self.last_time = None

        # Suscripciones
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        self.sub_goal = self.create_subscription(
            Goal, 'goal', self.goal_callback, 10)

        # Publishers
        self.pub_cmd          = self.create_publisher(Twist, 'cmd_vel', qos.qos_profile_sensor_data)
        self.pub_goal_reached = self.create_publisher(Bool, 'goal_reached', 10)

        self.timer = self.create_timer(0.05, self.run)

        self.get_logger().info("Controller Node Started. Waiting for goal...")

    def goal_callback(self, msg):
        if not msg.is_reachable:
            self.get_logger().warn("Received unreachable goal, ignoring.")
            return

        self.current_goal = msg
        self.phase        = 'rotate'
        self.V_actual     = 0.0
        self.finished     = False
        self.returning_home = False
        self.get_logger().info(f"New goal received: ({msg.x:.2f}, {msg.y:.2f}, θ={msg.theta:.2f})")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def apply_linear_deadzone(self, v):
        if abs(v) < 1e-3:
            return 0.0
        return np.clip(abs(v), self.V_min, self.V_max) * np.sign(v)

    def run(self):
        now = self.get_clock().now()
        if self.first:
            self.last_time = now
            self.first = False
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Sin goal todavía
        if self.phase == 'wait' or self.current_goal is None:
            return

        if self.finished:
            self.stop_robot()
            return

        # ── FASE FINAL: ORIENTACIÓN DEL GOAL ──────────────────────
        if self.returning_home:
            e_theta_final = wrap_to_pi(self.current_goal.theta - self.theta)
            cmd = Twist()
            if abs(e_theta_final) > self.align_threshold:
                cmd.angular.z = self.W_fixed * np.sign(e_theta_final)
                cmd.linear.x  = 0.0
                self.pub_cmd.publish(cmd)
            else:
                self.get_logger().info("Final orientation reached. Goal complete!")
                self.returning_home = False
                self.finished       = True
                self.stop_robot()
                reached = Bool()
                reached.data = True
                self.pub_goal_reached.publish(reached)
            return

        x_g = self.current_goal.x
        y_g = self.current_goal.y

        e_x     = x_g - self.x
        e_y     = y_g - self.y
        e_d     = np.sqrt(e_x**2 + e_y**2)
        e_theta = wrap_to_pi(np.arctan2(e_y, e_x) - self.theta)

        cmd = Twist()

        # ── FASE 1: ROTAR ──────────────────────────────────────────
        if self.phase == 'rotate':
            if abs(e_theta) > self.align_threshold:
                cmd.angular.z = self.W_fixed * np.sign(e_theta)
                cmd.linear.x  = 0.0
            else:
                self.get_logger().info("Aligned! Advancing...")
                self.phase    = 'advance'
                self.V_actual = 0.0

        # ── FASE 2: AVANZAR ────────────────────────────────────────
        if self.phase == 'advance':
            if e_d < self.goal_threshold:
                # Frenar con rampa
                self.V_actual = max(self.V_actual - self.max_accel * dt, 0.0)
                cmd.linear.x  = self.apply_linear_deadzone(self.V_actual)
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)

                if e_d < self.goal_reached_d and self.V_actual < 1e-3:
                    self.get_logger().info(f"Position reached! Adjusting orientation...")
                    self.returning_home = True
                    self.V_actual       = 0.0
                return

            V_deseada = np.clip(self.Kv * e_d, self.V_min, self.V_max)

            if V_deseada > self.V_actual:
                self.V_actual = min(self.V_actual + self.max_accel * dt, V_deseada)
            else:
                self.V_actual = max(self.V_actual - self.max_accel * dt, V_deseada)

            cmd.linear.x  = self.apply_linear_deadzone(self.V_actual)
            cmd.angular.z = self.Kw * e_theta

        self.pub_cmd.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt! Stopping...")
        self.stop_robot()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopController()
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
