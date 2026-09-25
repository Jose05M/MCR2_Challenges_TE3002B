#!/usr/bin/env python3
"""
Pose Controller - MCR2 Mini Challenge 1
Cumple con: Robustez (Perfil Trapezoidal + Corrección de Fricción) y FSM estricta.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from trajectory_msgs_custom.msg import TrajectoryPose

class PoseController(Node):

    # FSM States
    STATE_IDLE = 'IDLE'
    STATE_TURN = 'TURN'
    STATE_STRAIGHT = 'STRAIGHT'

    def __init__(self):
        super().__init__('path_controller')

        # Parámetros físicos
        self.declare_parameter('v_max', 0.38)
        self.declare_parameter('w_max', 4.85)
        self.declare_parameter('dist_corr', 1.05) # Compensación de slip
        self.declare_parameter('turn_corr', 1.10) # Compensación de inercia

        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.dist_corr = self.get_parameter('dist_corr').value
        self.turn_corr = self.get_parameter('turn_corr').value

        # Estado e integradores
        self.state = self.STATE_IDLE
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.seg = {}

        # ROS
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_done = self.create_publisher(Bool, '/path_done', 10)
        self.sub_pose = self.create_subscription(TrajectoryPose, '/pose', self._on_pose_cb, 10)
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info('Controlador FSM robusto inicializado.')

    def _on_pose_cb(self, msg: TrajectoryPose):
        if self.state != self.STATE_IDLE:
            return

        # 1. Matemática Pura (Sin factor de corrección)
        dx = msg.x - self.current_x
        dy = msg.y - self.current_y
        dist = math.hypot(dx, dy)
        
        target_angle = math.atan2(dy, dx)
        delta_theta = self._normalize_angle(target_angle - self.current_theta)

        # Auto-Tuning
        t_turn = msg.t_arrival * 0.3     	  # Asignamos 30% del tiempo a girar
        t_straight = msg.t_arrival * 0.7 	  # 70% a avanzar
        
        w_cruise = abs(delta_theta) / t_turn if t_turn > 0 else 0.0
        v_cruise = dist / t_straight if t_straight > 0 else 0.0

        # Limitación física por seguridad
        w_cruise = min(w_cruise, self.w_max)
        v_cruise = min(v_cruise, self.v_max)

        self.seg = {
            'w': math.copysign(w_cruise, delta_theta),
            'v': v_cruise,
            't_turn': abs(delta_theta) / w_cruise if w_cruise > 0 else 0,
            't_straight': dist / v_cruise if v_cruise > 0 else 0
        }

        self.phase_start = self.get_clock().now()
        self.state = self.STATE_TURN if abs(delta_theta) > 0.02 else self.STATE_STRAIGHT

    def _loop(self):
    
        now = self.get_clock().now()
        real_dt = (now - self.last_time).nanoseconds / 1e9
        
        self.last_time = now

        if self.state == self.STATE_IDLE:
            return

        elapsed = (now - self.phase_start).nanoseconds / 1e9
        cmd = Twist()

        if self.state == self.STATE_TURN:
            if elapsed < self.seg['t_turn']:

                cmd.angular.z = self.seg['w'] * self.turn_corr
                self.current_theta += self.seg['w'] * real_dt
            else:
                self.state = self.STATE_STRAIGHT
                self.phase_start = now
                
        elif self.state == self.STATE_STRAIGHT:
            if elapsed < self.seg['t_straight']:

                cmd.linear.x = self.seg['v'] * self.dist_corr
                
                self.current_x += self.seg['v'] * math.cos(self.current_theta) * real_dt
                self.current_y += self.seg['v'] * math.sin(self.current_theta) * real_dt
                
            else:
                self._finish_segment()
                
        self.pub_cmd.publish(cmd)

    def _finish_segment(self):
        self.pub_cmd.publish(Twist()) # Stop robot
        self.pub_done.publish(Bool(data=True))
        self.state = self.STATE_IDLE
        self.get_logger().info(f"Segmento completado. Odometría: X={self.current_x:.2f}, Y={self.current_y:.2f}")

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PoseController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
