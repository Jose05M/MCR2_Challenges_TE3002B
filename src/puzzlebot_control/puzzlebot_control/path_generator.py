import rclpy
import signal
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Bool
from puzzlebot_msgs.msg import Goal


# Velocidad máxima lineal y angular del robot (para validar alcanzabilidad)
V_MAX = 0.40
W_MAX = 1.0
# Distancia mínima que el robot puede alcanzar con control estable
MIN_REACHABLE_DIST = 0.05


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        # Lista de waypoints: (x, y, theta_final)
        # Cuadrado de 0.5m + regreso a orientación inicial
        self.waypoints = [
            (0.5, 0.0,  0.0),
            (0.5, 0.5,  np.pi / 2),
            (0.0, 0.5,  np.pi),
            (0.0, 0.0,  0.0),   # regresa al origen con orientación inicial
        ]
        self.goal_idx = 0
        self.waiting  = False   # esperando confirmación del controlador

        # Publisher y suscripción
        self.pub_goal = self.create_publisher(Goal, 'goal', 10)

        self.sub_reached = self.create_subscription(
            Bool, 'goal_reached', self.reached_callback, 10)

        # Timer de arranque — espera 1s para que el controlador esté listo
        self.startup_timer = self.create_timer(1.0, self.send_first_goal)

        self.get_logger().info("Path Generator Node Started.")

    def is_reachable(self, x, y, theta):
        """
        Verifica si el punto es alcanzable.
        Criterios:
        - Distancia mínima > MIN_REACHABLE_DIST
        - El ángulo theta está en [-pi, pi]
        """
        dist = np.sqrt(x**2 + y**2)
        if dist < MIN_REACHABLE_DIST:
            self.get_logger().warn(f"Point ({x}, {y}) too close to origin, unreachable.")
            return False
        if abs(theta) > np.pi:
            self.get_logger().warn(f"Theta {theta:.2f} out of range [-pi, pi], unreachable.")
            return False
        return True

    def send_first_goal(self):
        # Este timer solo corre una vez
        self.startup_timer.cancel()
        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints completed! Mission finished.")
            return

        x, y, theta = self.waypoints[self.goal_idx]
        reachable   = self.is_reachable(x, y, theta)

        msg              = Goal()
        msg.x            = float(x)
        msg.y            = float(y)
        msg.theta        = float(theta)
        msg.is_reachable = reachable

        self.pub_goal.publish(msg)
        self.waiting = True

        if reachable:
            self.get_logger().info(
                f"Sending goal {self.goal_idx + 1}/{len(self.waypoints)}: "
                f"({x:.2f}, {y:.2f}, θ={np.degrees(theta):.1f}°)")
        else:
            self.get_logger().warn(
                f"Goal {self.goal_idx + 1} marked as unreachable, skipping.")
            self.goal_idx += 1
            self.waiting   = False
            self.send_next_goal()

    def reached_callback(self, msg):
        if msg.data and self.waiting:
            self.get_logger().info(f"Goal {self.goal_idx + 1} confirmed reached!")
            self.goal_idx += 1
            self.waiting   = False
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
