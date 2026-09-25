#!/usr/bin/env python3
"""
Path Generator - MCR2 Mini Challenge 1
Cumple con: Validación de factibilidad, Auto-tuning de velocidades y lectura de parámetros.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs_custom.msg import TrajectoryPose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # ── Parámetros del Path (Definidos por el usuario) ──
        self.declare_parameter('points_x', [2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('points_y', [0.0, 2.0, 2.0, 0.0])
        self.declare_parameter('points_theta', [1.57, 3.14, -1.57, 0.0])
        self.declare_parameter('points_t', [8.0, 8.0, 8.0, 8.0])
        
        # Límites para validación (Reachability)
        self.declare_parameter('v_max', 0.38)
        self.declare_parameter('w_max', 4.85)

        px = self.get_parameter('points_x').value
        py = self.get_parameter('points_y').value
        pth = self.get_parameter('points_theta').value
        pt = self.get_parameter('points_t').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value

        self.waypoints = [{'x': px[i], 'y': py[i], 'theta': pth[i], 't': pt[i]} for i in range(len(px))]
        self.index = 0
        self.current_x = 0.0
        self.current_y = 0.0

        # ── ROS ──
        self.pub_pose = self.create_publisher(TrajectoryPose, '/pose', 10)
        self.sub_done = self.create_subscription(Bool, '/path_done', self._on_done_cb, 10)

        self.get_logger().info('Generador listo. Evaluando factibilidad de la trayectoria...')
        
        # CORRECCIÓN: El timer apunta a una función que envía y luego se destruye
        self.timer = self.create_timer(1.0, self._send_first)

    def _send_first(self):
        """Se ejecuta una sola vez al inicio para arrancar la secuencia."""
        self.timer.cancel()  # Destruye el timer para que NO se repita cada segundo
        self._publish_next()

    def _validate_reachability(self, wp):
        """Calcula si el punto es alcanzable (Reachability Check)"""
        dx = wp['x'] - self.current_x
        dy = wp['y'] - self.current_y
        dist = math.hypot(dx, dy)
        
        # Tiempo mínimo físico requerido
        t_min_straight = dist / self.v_max if dist > 0 else 0.0
        
        if wp['t'] < t_min_straight:
            self.get_logger().error(
                f"Punto inalcanzable. Se requieren al menos {t_min_straight:.2f}s "
                f"para recorrer {dist:.2f}m a {self.v_max}m/s. T_dado: {wp['t']}s"
            )
            return False
            
        # Estimación de velocidad requerida (Auto-tuning)
        v_req = dist / wp['t'] if wp['t'] > 0 else 0.0
        self.get_logger().info(f"Punto {self.index + 1} alcanzable. Vel. lineal requerida: {v_req:.3f} m/s")
        return True

    def _on_done_cb(self, msg: Bool):
        # CORRECCIÓN: Solo avanzar si el controlador dice "True" (terminé)
        if msg.data:
            self.get_logger().info('Controlador confirma llegada. Procesando siguiente...')
            self._publish_next()

    def _publish_next(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('Trayectoria completada.')
            return

        wp = self.waypoints[self.index]

        if not self._validate_reachability(wp):
            self.get_logger().warn("Abortando trayectoria por violación de límites físicos.")
            return

        msg = TrajectoryPose()
        msg.x = float(wp['x'])
        msg.y = float(wp['y'])
        msg.theta = float(wp['theta'])
        msg.t_arrival = float(wp['t'])

        self.pub_pose.publish(msg)
        self.get_logger().info(f'Enviando Punto {self.index + 1} (X: {msg.x}, Y: {msg.y}) al controlador.')
        
        # Actualizar posición para el cálculo del siguiente segmento
        self.current_x = wp['x']
        self.current_y = wp['y']
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathGenerator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()







def _validate_reachability(self, wp):
    """Calcula si el punto es alcanzable (Reachability Check)"""
    # 1. Calculamos la distancia al objetivo
    dist = math.hypot(wp['x'] - self.current_x, wp['y'] - self.current_y)
    
    # 2. Verificamos el límite físico del robot
    tiempo_minimo = dist / self.v_max 
    
    # 3. Abortamos si el tiempo solicitado es inalcanzable
    if wp['t'] < tiempo_minimo:
        return False
        
    return True
    
    
    
