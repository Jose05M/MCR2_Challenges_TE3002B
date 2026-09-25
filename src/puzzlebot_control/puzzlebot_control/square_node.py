import rclpy
import numpy as np
import signal

from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
        result += 2 * np.pi
    return result - np.pi


class ClosedLoopController(Node):

    def __init__(self):
        super().__init__('square_node')

        # Ganancias
        self.Kv = self.declare_parameter('Kv', 0.7).value
        self.Kw = self.declare_parameter('Kw', 0.6).value

        # Rampa lineal
        self.V_actual = 0.0
        self.max_accel = 0.4       # m/s²

        # Límites de velocidad
        self.V_min = 0.06          # deadzone lineal
        self.V_max = 0.40          # máximo lineal
        self.W_fixed = 1.0         # velocidad angular fija al girar

        # Umbral de alineación para pasar de fase 1 a fase 2
        self.align_threshold = 0.05  # rad (~3°)

        # Threshold de llegada
        self.goal_threshold = 0.05  # metros
        self.goal_reached   = 0.06	

        # Waypoints del cuadrado
        self.waypoints = [
            (1.0, 0.5),
            (1.0, 1.0),
            (0.0, 1.5),
            (0.0, 0.0),
        ]
        self.goal_idx = 0
        self.finished = False

        # Fase actual: 'rotate' | 'advance'
        self.phase = 'rotate'

        # Pose actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # dt
        self.first = True
        self.last_time = None

        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        self.pub_cmd = self.create_publisher(
            Twist, 'cmd_vel', qos.qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.run)  # 20 Hz

        self.get_logger().info("Square node started.")
        self.get_logger().info(f"Goal 1/{len(self.waypoints)}: {self.waypoints[0]}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def apply_linear_deadzone(self, v):
        """Nunca manda velocidad lineal en la zona muerta (0, V_min)"""
        if abs(v) < 1e-3:
            return 0.0                          # parada intencional
        return np.clip(abs(v), self.V_min, self.V_max) * np.sign(v)

    def run(self):

        now = self.get_clock().now()
        if self.first:
            self.last_time = now
            self.first = False
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.finished:
            self.stop_robot()
            return

        x_g, y_g = self.waypoints[self.goal_idx]

        e_x = x_g - self.x
        e_y = y_g - self.y
        e_d = np.sqrt(e_x**2 + e_y**2)
        e_theta = wrap_to_pi(np.arctan2(e_y, e_x) - self.theta)

        cmd = Twist()

        # ── FASE 1: ROTAR ──────────────────────────────────────────
        if self.phase == 'rotate':
            if abs(e_theta) > self.align_threshold:
                # Velocidad angular fija, signo según dirección del error
                cmd.angular.z = self.W_fixed * np.sign(e_theta)
                cmd.linear.x  = 0.0
            else:
                # Alineado — pasar a fase avance
                self.get_logger().info(f"Aligned! Switching to advance phase.")
                self.phase = 'advance'
                self.V_actual = 0.0  # arranca desde 0 con rampa

        # ── FASE 2: AVANZAR ────────────────────────────────────────
        if self.phase == 'advance':

            # Llegada al waypoint
            if e_d < self.goal_threshold:
                # Frena con rampa hasta 0
                self.V_actual = max(self.V_actual - self.max_accel * dt, 0.0)
                cmd.linear.x  = self.apply_linear_deadzone(self.V_actual)
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)

                if e_d < self.goal_reached and self.V_actual < 1e-3:
                    self.get_logger().info(f"Goal {self.goal_idx + 1} reached!")
                    self.goal_idx += 1
                    self.V_actual = 0.0

                    if self.goal_idx >= len(self.waypoints):
                        self.get_logger().info("All goals completed!")
                        self.finished = True
                        self.stop_robot()
                    else:
                        self.get_logger().info(
                            f"Goal {self.goal_idx + 1}/{len(self.waypoints)}: {self.waypoints[self.goal_idx]}")
                        self.phase = 'rotate'  # vuelve a alinear para el siguiente
                return

            # Velocidad deseada con rampa
            V_deseada = np.clip(self.Kv * e_d, self.V_min, self.V_max)

            if V_deseada > self.V_actual:
                self.V_actual = min(self.V_actual + self.max_accel * dt, V_deseada)
            else:
                self.V_actual = max(self.V_actual - self.max_accel * dt, V_deseada)

            cmd.linear.x  = self.apply_linear_deadzone(self.V_actual)
            # Corrección angular suave mientras avanza (por si se desvía)
            cmd.angular.z = self.Kw * e_theta

        self.pub_cmd.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping robot...")
        self.stop_robot()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopController()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
