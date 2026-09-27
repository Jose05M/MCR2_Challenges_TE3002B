#!/usr/bin/env python3
"""
Puzzlebot controller (final challenge motion layer).

Executes the MotionCommand sent by pzb_fsm/fsm_node on /motion_command and
is the only node that publishes /cmd_vel:

  FOLLOW_LINE  line-following PD on /line_error (adaptive Kp, speed ramp,
               slow-down on curves); linear_scale / angular_scale multiply
               the output (0, 0 keeps the PD running but outputs zero)
  STOP         zero velocity (an unfinished DRIVE / ROTATE is paused)
  DRIVE        go straight `distance` metres at `speed`, measured with /odom
  ROTATE       turn `angle` radians in place, measured with /odom

DRIVE and ROTATE publish True on /motion_done when they finish.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import numpy as np
from nav_msgs.msg import Odometry
from rclpy import qos

from pzb_interfaces.msg import MotionCommand


class PuzzlebotController(Node):
    """Line-following PD and odometry manoeuvres driven by MotionCommand."""

    def __init__(self):
        super().__init__('puzzlebot_controller')

        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('curve_speed', 0.10)
        self.declare_parameter('curve_threshold', 60.0)

        self.declare_parameter('kp_straight', 0.0012)
        self.declare_parameter('kp_curve', 0.0040)
        self.declare_parameter('kd', 0.0030)

        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('alpha', 0.5)

        # ROTATE: P controller on the heading error
        self.declare_parameter('rotate_kp', 0.8)
        self.declare_parameter('rotate_max_w', 1.5)          # [rad/s]
        self.declare_parameter('rotate_tolerance_deg', 5.0)  # [deg]

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('line_error_topic', '/line_error')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('motion_command_topic', '/motion_command')
        self.declare_parameter('motion_done_topic', '/motion_done')

        self.linear_speed = self.get_parameter('linear_speed').value
        self.curve_speed = self.get_parameter('curve_speed').value
        self.curve_threshold = self.get_parameter('curve_threshold').value

        self.kp_straight = self.get_parameter('kp_straight').value
        self.kp_curve = self.get_parameter('kp_curve').value
        self.kd = self.get_parameter('kd').value

        self.w_max = self.get_parameter('max_angular_vel').value
        self.alpha = self.get_parameter('alpha').value

        self.rotate_kp = self.get_parameter('rotate_kp').value
        self.rotate_max_w = self.get_parameter('rotate_max_w').value
        self.rotate_tolerance = np.deg2rad(self.get_parameter('rotate_tolerance_deg').value)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        line_topic = self.get_parameter('line_error_topic').value
        odometry_topic = self.get_parameter('odom_topic').value
        command_topic = self.get_parameter('motion_command_topic').value
        done_topic = self.get_parameter('motion_done_topic').value

        # State
        self.line_error = 0.0
        self.prev_error = 0.0
        self.current_linear_vel = 0.0
        self.acceleration = 0.01   # speed ramp step
        self.curve_state = False
        self.line_lost_threshold = 140
        self.filtered_error = 0.0

        # Odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0

        # Current command. None = STOP until the FSM sends something
        self.command = None
        # Running DRIVE / ROTATE: key, start point, target heading, paused flag
        self.manoeuvre = None
        # Key of the last finished DRIVE / ROTATE (ignore it if the FSM repeats it)
        self.completed_key = None

        # ROS I/O
        self.pub_vel = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_done = self.create_publisher(Bool, done_topic, 10)
        self.sub_line = self.create_subscription(Float32, line_topic, self._line_callback, 1)
        self.sub_odom = self.create_subscription(
            Odometry, odometry_topic, self._odom_callback, qos.qos_profile_sensor_data)
        self.sub_command = self.create_subscription(
            MotionCommand, command_topic, self._command_callback, 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('PuzzlebotController Started')

    # Callbacks
    def _line_callback(self, msg: Float32):
        self.filtered_error = (self.alpha * self.filtered_error + (1 - self.alpha) * msg.data)
        self.line_error = self.filtered_error

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Quaternion → yaw (rotation around Z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = np.arctan2(siny_cosp, cosy_cosp)

    def _command_callback(self, msg: MotionCommand):
        if msg.mode in (MotionCommand.DRIVE, MotionCommand.ROTATE):
            key = (msg.mode, round(msg.distance, 4), round(msg.speed, 4), round(msg.angle, 4))
            if self.manoeuvre is not None and self.manoeuvre['key'] == key:
                self.manoeuvre['paused'] = False        # same command: continue it
            elif key != self.completed_key:
                self.start_manoeuvre(msg, key)          # new command: start it
        elif msg.mode == MotionCommand.STOP:
            if self.manoeuvre is not None:
                self.manoeuvre['paused'] = True         # keep its progress
        else:  # FOLLOW_LINE
            self.manoeuvre = None
            self.completed_key = None
        self.command = msg

    def start_manoeuvre(self, msg, key):
        # normalise
        target = self.current_heading + msg.angle
        self.manoeuvre = {
            'key': key,
            'start_x': self.current_x,
            'start_y': self.current_y,
            'target_heading': np.arctan2(np.sin(target), np.cos(target)),
            'paused': False,
        }
        self.completed_key = None

    def finish_manoeuvre(self):
        # reset the line-following PD
        self.prev_error = 0.0
        self.line_error = 0.0
        self.completed_key = self.manoeuvre['key']
        self.manoeuvre = None
        self.pub_done.publish(Bool(data=True))

    def handle_drive(self, distance, speed):
        # travelled distance
        m = self.manoeuvre
        travelled = np.sqrt((self.current_x - m['start_x'])**2
                            + (self.current_y - m['start_y'])**2)

        self.twist.linear.x = speed
        self.twist.angular.z = 0.0

        if travelled >= distance:
            self.twist.linear.x = 0.0
            self.finish_manoeuvre()

    def handle_rotate(self):
        heading_error = np.arctan2(
            np.sin(self.manoeuvre['target_heading'] - self.current_heading),
            np.cos(self.manoeuvre['target_heading'] - self.current_heading)
        )

        angular_vel = self.rotate_kp * heading_error
        angular_vel = np.clip(angular_vel, -self.rotate_max_w, self.rotate_max_w)
        self.twist.linear.x = 0.0
        self.twist.angular.z = angular_vel

        # rotation finished
        if abs(heading_error) < self.rotate_tolerance:
            self.twist.angular.z = 0.0
            self.finish_manoeuvre()

    def handle_follow_mode(self, speed_multiplier=1.0, angular_scale=1.0):

        # CURVE DETECTION
        curve_detected = (abs(self.line_error) > self.curve_threshold)
        if curve_detected != self.curve_state:
            if curve_detected:
                self.get_logger().info("↩️ Curve detected")
            else:
                self.get_logger().info("➡️ Straight section detected")
            self.curve_state = curve_detected

        # ADAPTIVE KP
        if curve_detected:
            kp = self.kp_curve
        else:
            kp = self.kp_straight

        # PD Controller
        derivative = self.line_error - self.prev_error
        derivative = np.clip(derivative, -70, 70)

        angular_vel = (kp * self.line_error + self.kd * derivative)
        self.prev_error = self.line_error

        # Saturation
        angular_vel = np.clip(angular_vel, -self.w_max, self.w_max)
        angular_vel *= 0.85

        # Adaptive Linear Speed
        if curve_detected:
            target_linear_vel = self.curve_speed
        else:
            target_linear_vel = self.linear_speed

        # -------- Acceleration Ramp --------
        if self.current_linear_vel < target_linear_vel:
            self.current_linear_vel += self.acceleration
            self.current_linear_vel = min(self.current_linear_vel, target_linear_vel)

        elif self.current_linear_vel > target_linear_vel:
            self.current_linear_vel -= self.acceleration
            self.current_linear_vel = max(self.current_linear_vel, target_linear_vel)

        # LINE LOST
        linear_vel = self.current_linear_vel
        if abs(self.line_error) > self.line_lost_threshold:
            linear_vel *= 0.4

        # TURN-BASED SPEED REDUCTION
        turn_factor = 1.0 - min(abs(angular_vel) / self.w_max, 1.0)
        turn_factor = max(turn_factor, 0.35)
        linear_vel *= turn_factor

        # SPECIAL MODIFIER
        linear_vel *= speed_multiplier

        self.twist.linear.x = linear_vel
        self.twist.angular.z = angular_vel * angular_scale

    def _control_loop(self):
        self.twist = Twist()
        cmd = self.command

        if cmd is None or cmd.mode == MotionCommand.STOP:
            pass

        elif cmd.mode == MotionCommand.FOLLOW_LINE:
            self.handle_follow_mode(cmd.linear_scale, cmd.angular_scale)

        elif self.manoeuvre is not None and not self.manoeuvre['paused']:
            if cmd.mode == MotionCommand.DRIVE:
                self.handle_drive(cmd.distance, cmd.speed)
            elif cmd.mode == MotionCommand.ROTATE:
                self.handle_rotate()

        self.pub_vel.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
